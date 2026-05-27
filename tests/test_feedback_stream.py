"""Unit tests for AsyncFeedbackStream's frame loop, counters, and reconnect.

Drives the async reader with a fake StreamReader feeding raw bytes — no real
socket. Needs numpy to build valid/bad-magic frames (reuses Phase 0's
``_make_frame``), so the whole module skips when numpy is unavailable.
"""

import asyncio
import unittest

try:
    import numpy  # noqa: F401

    from test_feedback import _make_frame  # reuse Phase 0 frame builder
    from robot_core.transport.feedback_stream import AsyncFeedbackStream

    HAVE_DEPS = True
except ImportError:  # pragma: no cover - depends on environment
    HAVE_DEPS = False


class FakeStreamReader:
    """Feeds scripted byte chunks via readexactly; raises IncompleteReadError at EOF."""

    def __init__(self, chunks):
        self._chunks = list(chunks)

    async def readexactly(self, n):
        if not self._chunks:
            raise asyncio.IncompleteReadError(partial=b"", expected=n)
        return self._chunks.pop(0)


class FakeWriter:
    def close(self):
        pass

    async def wait_closed(self):
        pass


@unittest.skipUnless(HAVE_DEPS, "numpy not installed")
class AsyncFeedbackStreamTests(unittest.IsolatedAsyncioTestCase):
    def _stream(self, readers, *, reconnect, **kwargs):
        """Build a stream whose opener hands out the given readers in order."""
        opened = []

        async def opener(host, port):
            reader = readers[len(opened)] if len(opened) < len(readers) else FakeStreamReader([])
            opened.append(reader)
            return reader, FakeWriter()

        stream = AsyncFeedbackStream(
            "192.0.2.1", 30004, open_connection=opener, reconnect=reconnect, **kwargs
        )
        return stream, opened

    async def test_skips_and_counts_invalid_frame(self):
        chunks = [
            _make_frame(robot_mode=1),
            _make_frame(test_value=0xDEADBEEF),  # intact length, bad magic
            _make_frame(robot_mode=2),
        ]
        stream, _ = self._stream([FakeStreamReader(chunks)], reconnect=False)
        gen = stream.frames()
        modes = [(await gen.__anext__()).robot_mode for _ in range(2)]
        await gen.aclose()

        self.assertEqual(modes, [1, 2])  # the bad frame was skipped, not yielded
        self.assertEqual(stream.invalid_frame_count, 1)

    async def test_propagates_when_reconnect_disabled(self):
        stream, _ = self._stream(
            [FakeStreamReader([_make_frame(robot_mode=1)])], reconnect=False
        )
        gen = stream.frames()
        self.assertEqual((await gen.__anext__()).robot_mode, 1)
        with self.assertRaises(asyncio.IncompleteReadError):
            await gen.__anext__()  # EOF after the single frame
        self.assertEqual(stream.incomplete_read_count, 1)
        await gen.aclose()

    async def test_reconnects_after_connection_loss(self):
        # First connection yields frame 1 then EOF; second yields frame 2.
        readers = [
            FakeStreamReader([_make_frame(robot_mode=1)]),
            FakeStreamReader([_make_frame(robot_mode=2)]),
        ]
        stream, opened = self._stream(readers, reconnect=True, retry_backoff_s=0)
        gen = stream.frames()
        first = (await gen.__anext__()).robot_mode
        second = (await gen.__anext__()).robot_mode  # forces reconnect to reader 2
        await gen.aclose()
        await stream.close()

        self.assertEqual([first, second], [1, 2])
        self.assertGreaterEqual(stream.incomplete_read_count, 1)
        self.assertGreaterEqual(len(opened), 2)  # reconnected at least once


if __name__ == "__main__":
    unittest.main()
