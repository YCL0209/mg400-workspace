"""Unit tests for RobotState edge-triggered notifications and subscriptions.

Operates on FeedbackFrame objects directly, so no numpy and no sockets.
"""

import asyncio
import unittest

from robot_core.state.robot_state import WATCHED_FIELDS, RobotState
from robot_core.transport.feedback import FeedbackFrame


def _frame(
    mode=5,
    enable=1,
    error=0,
    pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
):
    return FeedbackFrame(
        robot_mode=mode,
        enable_status=enable,
        error_status=error,
        tool_vector_actual=pose,
        q_actual=q,
    )


class EdgeTriggerTests(unittest.TestCase):
    def setUp(self):
        self.state = RobotState()
        self.events: list[tuple] = []
        self.unsub = self.state.subscribe(
            lambda snap, changed: self.events.append((snap, changed))
        )

    def test_first_frame_reports_all_watched_fields(self):
        self.state.update(_frame())
        self.assertEqual(len(self.events), 1)
        _, changed = self.events[0]
        self.assertEqual(changed, frozenset(WATCHED_FIELDS))

    def test_identical_frame_does_not_notify(self):
        self.state.update(_frame(mode=5, pose=(1, 2, 3, 4, 0, 0)))
        self.events.clear()
        self.state.update(_frame(mode=5, pose=(1, 2, 3, 4, 0, 0)))  # byte-identical
        self.assertEqual(self.events, [])

    def test_only_changed_field_is_reported(self):
        self.state.update(_frame(mode=5, error=0))
        self.events.clear()
        self.state.update(_frame(mode=5, error=1))  # only error_status flips
        self.assertEqual(len(self.events), 1)
        _, changed = self.events[0]
        self.assertEqual(changed, frozenset({"error_status"}))

    def test_pose_change_is_detected(self):
        self.state.update(_frame(pose=(0, 0, 0, 0, 0, 0)))
        self.events.clear()
        self.state.update(_frame(pose=(1.0, 0, 0, 0, 0, 0)))
        _, changed = self.events[0]
        self.assertEqual(changed, frozenset({"tool_vector_actual"}))

    def test_pose_jitter_within_deadband_does_not_notify(self):
        # Default deadband is 0.1; a 0.01 move is noise and must not notify.
        self.state.update(_frame(pose=(10.0, 20.0, 30.0, 0, 0, 0)))
        self.events.clear()
        self.state.update(_frame(pose=(10.01, 20.0, 30.0, 0, 0, 0)))
        self.assertEqual(self.events, [])

    def test_pose_move_beyond_deadband_notifies(self):
        # A 0.5 move exceeds the 0.1 deadband and must register as a change.
        self.state.update(_frame(pose=(10.0, 20.0, 30.0, 0, 0, 0)))
        self.events.clear()
        self.state.update(_frame(pose=(10.5, 20.0, 30.0, 0, 0, 0)))
        self.assertEqual(len(self.events), 1)
        _, changed = self.events[0]
        self.assertEqual(changed, frozenset({"tool_vector_actual"}))

    def test_update_returns_changed_set(self):
        self.assertEqual(self.state.update(_frame(mode=5)), frozenset(WATCHED_FIELDS))
        self.assertEqual(self.state.update(_frame(mode=5)), frozenset())  # no change

    def test_snapshot_and_seq_advance(self):
        self.state.update(_frame(mode=5))
        self.state.update(_frame(mode=6))
        self.assertEqual(self.state.snapshot.robot_mode, 6)
        self.assertEqual(self.state.snapshot.seq, 2)
        self.assertTrue(self.state.snapshot.is_enabled)

    def test_unsubscribe_stops_callbacks(self):
        self.state.update(_frame(mode=1))
        self.unsub()
        self.events.clear()
        self.state.update(_frame(mode=2))
        self.assertEqual(self.events, [])

    def test_one_bad_subscriber_does_not_break_others(self):
        delivered = []

        def bad(snap, changed):
            raise RuntimeError("boom")

        self.state.subscribe(bad)
        self.state.subscribe(lambda snap, changed: delivered.append(changed))
        with self.assertLogs(level="ERROR"):  # the bad subscriber's error is logged
            self.state.update(_frame(mode=9))
        self.assertEqual(len(delivered), 1)


class WaitForChangeTests(unittest.IsolatedAsyncioTestCase):
    async def test_resolves_on_next_edge(self):
        state = RobotState()

        async def trigger():
            await asyncio.sleep(0.01)
            state.update(_frame(mode=7))

        task = asyncio.create_task(trigger())
        snapshot, changed = await state.wait_for_change(timeout=1.0)
        self.assertEqual(snapshot.robot_mode, 7)
        self.assertIn("robot_mode", changed)
        await task

    async def test_times_out_when_no_change(self):
        state = RobotState()
        with self.assertRaises(asyncio.TimeoutError):
            await state.wait_for_change(timeout=0.02)


if __name__ == "__main__":
    unittest.main()
