"""Unit tests for RobotStateMonitor and its drain-to-latest slot.

Uses a fake stream that yields FeedbackFrame objects, so no numpy and no
sockets. Covers: drain-to-latest semantics, state convergence to the newest
frame, counter pass-through, and clean cancellation / resource release.
"""

import asyncio
import unittest

from robot_core.state.monitor import RobotStateMonitor, _LatestFrameSlot
from robot_core.state.robot_state import RobotState
from robot_core.transport.feedback import FeedbackFrame


def _frame(mode):
    return FeedbackFrame(
        robot_mode=mode, enable_status=1, error_status=0,
        tool_vector_actual=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        q_actual=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )


class FakeStream:
    """Stand-in for AsyncFeedbackStream: yields scripted frames, then parks.

    After the scripted frames it awaits an Event that is never set, modelling a
    live stream waiting for more data — so the producer task stays alive until
    the monitor cancels it (which is what the cancellation tests exercise).
    """

    def __init__(self, frames):
        self._frames = list(frames)
        self.invalid_frame_count = 0
        self.incomplete_read_count = 0
        self.closed = False
        self._park = asyncio.Event()

    async def frames(self):
        for frame in self._frames:
            yield frame
        await self._park.wait()  # block until cancelled

    async def close(self):
        self.closed = True


class LatestFrameSlotTests(unittest.IsolatedAsyncioTestCase):
    async def test_keeps_latest_and_counts_skipped(self):
        slot = _LatestFrameSlot()
        slot.push(_frame(1))
        slot.push(_frame(2))
        slot.push(_frame(3))
        self.assertEqual(slot.skipped, 2)  # f1, f2 superseded before consumption
        self.assertEqual((await slot.get()).robot_mode, 3)  # newest survives

    async def test_no_skip_when_consumed_between_pushes(self):
        slot = _LatestFrameSlot()
        slot.push(_frame(1))
        self.assertEqual((await slot.get()).robot_mode, 1)
        slot.push(_frame(2))
        self.assertEqual((await slot.get()).robot_mode, 2)
        self.assertEqual(slot.skipped, 0)


class RobotStateMonitorTests(unittest.IsolatedAsyncioTestCase):
    async def test_state_converges_to_latest_frame(self):
        stream = FakeStream([_frame(1), _frame(2), _frame(3)])
        state = RobotState()
        monitor = RobotStateMonitor(stream, state)
        monitor.start()
        try:
            await asyncio.sleep(0.05)  # let producer/consumer drain
            self.assertEqual(state.snapshot.robot_mode, 3)
        finally:
            await monitor.stop()

    async def test_stop_cancels_tasks_and_closes_stream(self):
        stream = FakeStream([_frame(1)])
        monitor = RobotStateMonitor(stream, RobotState())
        monitor.start()
        await asyncio.sleep(0.01)
        self.assertTrue(monitor.is_running)

        await monitor.stop()

        self.assertFalse(monitor.is_running)
        self.assertTrue(stream.closed)
        self.assertIsNone(monitor._producer)
        self.assertIsNone(monitor._consumer)

    async def test_async_context_manager_runs_and_cleans_up(self):
        stream = FakeStream([_frame(5)])
        state = RobotState()
        async with RobotStateMonitor(stream, state):
            await asyncio.sleep(0.01)
            self.assertEqual(state.snapshot.robot_mode, 5)
        self.assertTrue(stream.closed)

    async def test_counters_pass_through_from_stream(self):
        stream = FakeStream([_frame(1)])
        stream.invalid_frame_count = 3
        stream.incomplete_read_count = 2
        monitor = RobotStateMonitor(stream, RobotState())
        self.assertEqual(monitor.invalid_frame_count, 3)
        self.assertEqual(monitor.incomplete_read_count, 2)
        monitor.start()
        try:
            await asyncio.sleep(0.02)
            self.assertGreaterEqual(monitor.stale_frame_count, 0)
        finally:
            await monitor.stop()


if __name__ == "__main__":
    unittest.main()
