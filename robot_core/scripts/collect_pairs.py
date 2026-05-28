"""Collect FK-calibration pairs from the live arm — passively, no vendor GUI.

Captures matched ``(joint angles, cartesian pose)`` samples straight from the
30004 feedback stream so forward-kinematics parameters can be calibrated using
only our own code (part of owning the read path, independent of DobotStudio).

PASSIVE BY DESIGN — this script only *reads*:
* Connects to 30004 (feedback) only. Never connects to the dashboard/move ports,
  never enables the arm, never sends any command.
* YOU drive the hardware by hand: enable + press the drag-teach button on the
  arm, pose it, let it settle, then capture.

Operating procedure:
  1. Power on the arm and (on the hardware) enable it and hold the drag-teach
     button. Move the arm to a posture by hand and let it come to rest.
  2. In this script, press Enter to capture the current frame. Optionally type a
     short label first (e.g. ``factory`` or ``reach-far``) then Enter.
  3. Repeat for several postures. Both the joint angles and the pose in each
     captured row come from the SAME feedback frame (same instant).
  4. Type ``q`` (then Enter), or Ctrl-D / Ctrl-C, to finish. All pairs are
     printed and written to ``outputs/pairs_<timestamp>.json``.

Run it::

    python -m robot_core.scripts.collect_pairs
    MG400_IP=192.168.1.20 python -m robot_core.scripts.collect_pairs

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

logger = logging.getLogger("robot_core.collect_pairs")

OUTPUT_DIR = Path(__file__).resolve().parent.parent.parent / "outputs"
FIRST_FRAME_TIMEOUT_S = 10.0


@dataclass
class Pair:
    """One captured posture: joints and pose from a single feedback frame."""

    index: int
    label: str
    # Joint angles J1..J4 (deg) and cartesian pose x/y/z (mm), r (deg).
    j1: float
    j2: float
    j3: float
    j4: float
    x: float
    y: float
    z: float
    r: float
    # Provenance, so a pair can be traced back to its frame.
    seq: int
    robot_mode: int
    is_enabled: bool
    has_error: bool
    captured_at: str
    # Raw 6-vectors as received, for full fidelity.
    q_actual: list
    tool_vector_actual: list

    @classmethod
    def from_snapshot(cls, index: int, label: str, snap: RobotStateSnapshot) -> "Pair":
        j1, j2, j3, j4 = snap.joints
        x, y, z, r = snap.tool_vector_actual[0:4]
        return cls(
            index=index,
            label=label,
            j1=j1, j2=j2, j3=j3, j4=j4,
            x=x, y=y, z=z, r=r,
            seq=snap.seq,
            robot_mode=snap.robot_mode,
            is_enabled=snap.is_enabled,
            has_error=snap.has_error,
            captured_at=datetime.now().isoformat(timespec="seconds"),
            q_actual=list(snap.q_actual),
            tool_vector_actual=list(snap.tool_vector_actual),
        )

    def one_line(self) -> str:
        tag = f" [{self.label}]" if self.label else ""
        return (
            f"#{self.index}{tag}  J=({self.j1:.3f}, {self.j2:.3f}, {self.j3:.3f}, "
            f"{self.j4:.3f})  pose=({self.x:.3f}, {self.y:.3f}, {self.z:.3f}, {self.r:.3f})"
        )


_PROMPT = "Enter=記錄 (可先打標籤) | q=結束 > "


async def collect(config: RobotConfig) -> list[Pair]:
    """Run the passive capture loop until the operator stops it."""
    stream = AsyncFeedbackStream(
        config.ip,
        config.feedback_port,
        connect_timeout_s=config.transport.connect_timeout_s,
        retry_backoff_s=config.transport.retry_backoff_s,
    )
    state = RobotState()
    monitor = RobotStateMonitor(stream, state)
    pairs: list[Pair] = []

    try:
        monitor.start()
        print(f"連線 feedback 串流 {config.ip}:{config.feedback_port}（純被動，只讀）...")
        await _await_first_frame(state)
        if state.snapshot is None:
            print("⚠ 逾時仍未收到 feedback 幀，請確認手臂已開機並在同網段。")
            return pairs

        print("已開始接收狀態。請在硬體上手動使能＋按住示教鈕拖曳，擺好靜止後按 Enter 記一筆。")
        # Read keyboard input on a worker thread so the feedback monitor keeps
        # running on the event loop. asyncio.to_thread works on Windows
        # (ProactorEventLoop) as well as macOS/Linux — unlike loop.add_reader,
        # which raises NotImplementedError for stdin on Windows.
        while True:
            try:
                line = await asyncio.to_thread(input, _PROMPT)
            except EOFError:  # Ctrl-D
                break
            command = line.strip()
            if command.lower() == "q":
                break
            # Snapshot this instant's frame: joints and pose come from one frame.
            snap = state.snapshot
            if snap is None:
                print("⚠ 尚未收到 feedback 幀，稍候再按 Enter")
                continue
            pair = Pair.from_snapshot(len(pairs) + 1, command, snap)
            pairs.append(pair)
            print("  記錄 " + pair.one_line())
        return pairs
    finally:
        # Runs for q / Ctrl-D / Ctrl-C (cancellation) alike: clean stop + save.
        await monitor.stop()
        _report(pairs, monitor)


async def _await_first_frame(state: RobotState) -> None:
    """Wait (briefly) for the first frame so the first capture has data."""
    loop = asyncio.get_running_loop()
    deadline = loop.time() + FIRST_FRAME_TIMEOUT_S
    while state.snapshot is None:
        if loop.time() > deadline:
            return
        await asyncio.sleep(0.05)


def _report(pairs: list[Pair], monitor: RobotStateMonitor) -> None:
    """Print every captured pair and write them to outputs/."""
    print("\n=== 採集到的配對 ===")
    if not pairs:
        print("（沒有任何配對）")
    for pair in pairs:
        print("  " + pair.one_line())
    print(
        f"frame 統計: invalid={monitor.invalid_frame_count} "
        f"incomplete={monitor.incomplete_read_count} stale_skipped={monitor.stale_frame_count}"
    )

    if not pairs:
        return
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUTPUT_DIR / f"pairs_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    payload = {
        "captured_at": datetime.now().isoformat(timespec="seconds"),
        "count": len(pairs),
        "note": "joints J1..J4 in deg; pose x/y/z in mm, r in deg; from 30004 feedback",
        "pairs": [asdict(p) for p in pairs],
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"\n已存 {len(pairs)} 筆到 {path}")


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    config = RobotConfig.load()
    try:
        asyncio.run(collect(config))
    except KeyboardInterrupt:
        # asyncio.run already unwound collect()'s finally (report + save) on Ctrl-C.
        print("\n(中止)")


if __name__ == "__main__":
    main()
