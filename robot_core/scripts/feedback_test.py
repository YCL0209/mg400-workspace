"""Phase 1 feedback observation test (passive).

Connects to the 30004 feedback stream **only** and observes the arm's state for
a fixed duration, printing RobotState *changes* (not every frame). It then shuts
down gracefully and prints frame statistics.

PASSIVE BY DESIGN:
* Connects to 30004 (feedback) only — never to 29999 (dashboard).
* Does NOT enable/power the arm and issues NO commands of any kind. The 30004
  stream pushes status regardless of enable state, so this is the safest, most
  layer-clean way to exercise the reader. "enable -> watch flip -> disable" is a
  separate later milestone and intentionally not mixed in here.

Run it::

    python -m robot_core.scripts.feedback_test
    python -m robot_core.scripts.feedback_test 10          # observe ~10 seconds
    MG400_IP=192.168.1.20 python -m robot_core.scripts.feedback_test

This is an operator-facing CLI: human-readable results go to ``print``, while
diagnostics/lifecycle go to ``logging`` (per CLAUDE.md's layered rule).
"""

from __future__ import annotations

import asyncio
import logging
import sys

from robot_core.config import RobotConfig
from robot_core.state import RobotState, RobotStateMonitor, RobotStateSnapshot
from robot_core.transport import AsyncFeedbackStream

logger = logging.getLogger("robot_core.feedback_test")

DEFAULT_OBSERVE_SECONDS = 5.0


def _on_change(snapshot: RobotStateSnapshot, changed: "frozenset[str]") -> None:
    """Print only what changed — edge-triggered, so it won't flood at 30004's rate."""
    pose = ", ".join(f"{v:.2f}" for v in snapshot.tool_vector_actual)
    print(
        f"[seq {snapshot.seq:>5}] changed={sorted(changed)} "
        f"mode={snapshot.robot_mode} enabled={snapshot.is_enabled} "
        f"error={snapshot.has_error} pose=({pose})"
    )


async def observe(config: RobotConfig, duration_s: float) -> int:
    """Run the passive observation for ``duration_s`` seconds. Returns exit code."""
    logger.info(
        "Passive feedback observation: %s:%d for %.1fs (no dashboard, no enable)",
        config.ip,
        config.feedback_port,
        duration_s,
    )

    stream = AsyncFeedbackStream(
        config.ip,
        config.feedback_port,
        connect_timeout_s=config.transport.connect_timeout_s,
        retry_backoff_s=config.transport.retry_backoff_s,
    )
    state = RobotState()
    unsubscribe = state.subscribe(_on_change)
    monitor = RobotStateMonitor(stream, state)

    try:
        monitor.start()
        print(f"--- observing for {duration_s:.1f}s (Ctrl-C to stop early) ---")
        await asyncio.sleep(duration_s)
        return 0
    except asyncio.CancelledError:
        logger.info("Observation cancelled")
        return 0
    finally:
        # Graceful shutdown: stop cancels the background tasks and closes the socket.
        unsubscribe()
        await monitor.stop()
        print("--- frame statistics ---")
        print(f"  invalid frames (bad magic) : {monitor.invalid_frame_count}")
        print(f"  incomplete reads (reconnect): {monitor.incomplete_read_count}")
        print(f"  stale frames skipped        : {monitor.stale_frame_count}")
        final = state.snapshot
        print(f"  final snapshot              : {final}")


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    duration = DEFAULT_OBSERVE_SECONDS
    if len(sys.argv) > 1:
        duration = float(sys.argv[1])

    config = RobotConfig.load()
    try:
        exit_code = asyncio.run(observe(config, duration))
    except KeyboardInterrupt:
        # asyncio.run already unwound observe()'s finally on Ctrl-C.
        exit_code = 0
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
