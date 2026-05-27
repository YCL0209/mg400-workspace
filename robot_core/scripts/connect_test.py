"""Phase 0 connection & safety baseline test.

Verifies the whole Phase 0 loop against a real MG400:

    connect (29999 + 30004)  ->  enable  ->  read status  ->  disable  ->  close

It issues **no motion commands** (no MovJ / MovL / Jog). Per CLAUDE.md, no
motion is allowed until Phase 0 passes.

Run it::

    python -m robot_core.scripts.connect_test
    MG400_IP=192.168.1.20 python -m robot_core.scripts.connect_test   # override IP

============================================================================
SAFETY — READ BEFORE RUNNING
============================================================================
* Enabling the robot energises the joints. Even with no motion command, the
  arm will hold position under power and may settle slightly. Make sure the
  arm is already in a safe, collision-free pose and the workspace is clear of
  hands and obstacles BEFORE running this script.
* Keep the physical emergency-stop button within reach. The script never
  moves the arm, but you remain the last line of defence.
* To trigger a software emergency stop, open a second terminal and run a
  ``FramedConnection`` against port 29999 sending ``EmergencyStop()`` — for
  example::

      python -c "from robot_core.config import RobotConfig; \
from robot_core.transport import FramedConnection; \
c=RobotConfig.load(); \
conn=FramedConnection(c.ip, c.dashboard_port); conn.connect(); \
print(conn.request('EmergencyStop()')); conn.close()"

  ``EmergencyStop()`` is designed to pre-empt the queue and must run on its own
  high-priority channel — never behind a queued command. A dedicated, always-
  available e-stop path is built in a later phase; for now it is manual.
* This script always disables the robot in a ``finally`` block, so any
  exception still routes through a safe power-down before exit.
============================================================================
"""

from __future__ import annotations

import logging
import sys

from robot_core.config import RobotConfig
from robot_core.transport import FramedConnection, TcpConnection, read_feedback_frame

logger = logging.getLogger("robot_core.connect_test")

# Phase 0 dashboard commands. Command-string assembly belongs in the protocol
# layer (CLAUDE.md), which does not exist yet — so the handful Phase 0 needs are
# named here as constants rather than scattered as inline literals.
CMD_ENABLE = "EnableRobot()"
CMD_DISABLE = "DisableRobot()"
CMD_ROBOT_MODE = "RobotMode()"
CMD_GET_POSE = "GetPose()"
CMD_GET_ANGLE = "GetAngle()"

# EnableRobot can take a few seconds to energise; give it a generous timeout.
ENABLE_TIMEOUT_S = 15.0


class DashboardCommandError(RuntimeError):
    """A dashboard command returned a non-zero ErrorID or an unparseable reply."""


def _check_ok(reply: str, command: str) -> str:
    """Raise unless the dashboard reply reports success, else return it unchanged.

    INTERIM guard: MG400 dashboard replies look like ``ErrorID,{value},Command();``
    where the leading integer is 0 on success and non-zero on failure. Phase 0 has
    no protocol layer yet, so this minimal check lives here; response parsing and
    command assembly move into the protocol layer in a later phase. Deliberately
    not a command framework — just "did this command actually succeed?".
    """
    head = reply.split(",", 1)[0].strip()
    try:
        error_id = int(head)
    except ValueError as exc:
        raise DashboardCommandError(
            f"{command}: malformed reply, no leading ErrorID: {reply!r}"
        ) from exc
    if error_id != 0:
        raise DashboardCommandError(
            f"{command}: controller returned ErrorID {error_id}: {reply!r}"
        )
    return reply


def run(config: RobotConfig | None = None) -> int:
    """Execute the Phase 0 sequence. Returns a process exit code (0 = success)."""
    config = config or RobotConfig.load()
    logger.info(
        "MG400 target ip=%s dashboard=%d feedback=%d",
        config.ip,
        config.dashboard_port,
        config.feedback_port,
    )

    dashboard = FramedConnection(
        config.ip,
        config.dashboard_port,
        connect_timeout_s=config.transport.connect_timeout_s,
        recv_timeout_s=config.transport.recv_timeout_s,
        max_retries=config.transport.max_retries,
        retry_backoff_s=config.transport.retry_backoff_s,
    )
    feedback = TcpConnection(
        config.ip,
        config.feedback_port,
        connect_timeout_s=config.transport.connect_timeout_s,
        recv_timeout_s=config.transport.recv_timeout_s,
        max_retries=config.transport.max_retries,
        retry_backoff_s=config.transport.retry_backoff_s,
    )

    enabled = False
    try:
        # 1) Connect to dashboard (29999) and feedback (30004).
        dashboard.connect()
        feedback.connect()

        # 2) Enable, then read status via dashboard queries.
        #    Gate `enabled` on the ErrorID: if EnableRobot did not actually
        #    succeed, raise before setting `enabled` / reading further — control
        #    falls through to the finally block for a safe close.
        logger.info("Enabling robot...")
        enable_reply = dashboard.request(CMD_ENABLE, timeout_s=ENABLE_TIMEOUT_S)
        _check_ok(enable_reply, CMD_ENABLE)
        enabled = True
        print("EnableRobot ->", enable_reply)

        print("RobotMode  ->", _check_ok(dashboard.request(CMD_ROBOT_MODE), CMD_ROBOT_MODE))
        print("GetPose    ->", _check_ok(dashboard.request(CMD_GET_POSE), CMD_GET_POSE))
        print("GetAngle   ->", _check_ok(dashboard.request(CMD_GET_ANGLE), CMD_GET_ANGLE))

        # 3) Read one frame off the 30004 stream, validate magic, print parsed state.
        frame = read_feedback_frame(feedback)
        print("--- feedback frame (validated) ---")
        print(f"  robot_mode         : {frame.robot_mode}")
        print(f"  enable_status      : {frame.enable_status} (enabled={frame.is_enabled})")
        print(f"  error_status       : {frame.error_status} (error={frame.has_error})")
        print(f"  tool_vector_actual : {frame.tool_vector_actual}")

        logger.info("Phase 0 sequence completed successfully.")
        return 0

    except Exception:  # noqa: BLE001 — top-level script boundary; log and power down.
        logger.exception("Phase 0 sequence failed")
        return 1

    finally:
        # 4) Safe power-down + close, no matter how we got here.
        try:
            if enabled and dashboard.is_connected:
                logger.info("Disabling robot...")
                disable_reply = dashboard.request(CMD_DISABLE)
                _check_ok(disable_reply, CMD_DISABLE)
                print("DisableRobot ->", disable_reply)
        except Exception:  # noqa: BLE001
            logger.exception("DisableRobot failed during cleanup")
        finally:
            dashboard.close()
            feedback.close()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    sys.exit(run())


if __name__ == "__main__":
    main()
