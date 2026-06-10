"""Tests for the M0c-2 FastAPI lifespan that wires arm RobotState into viz/.

The lifespan opens an AsyncFeedbackStream, drives a RobotState through a
RobotStateMonitor, and parks the resolved state in a closure-shared
holder (exposed on ``app.state.arm_state_holder`` for tests). When a ws
client connects, the default handeye_session_factory reads
``holder["state"]`` and injects it into the new HandeyeSession so the
operator sees ``ARM: ONLINE``.

Covered code paths:

1. connect succeeds -> arm_state populated -> holder reflects ONLINE
2. connect times out -> graceful ARM: OFFLINE (holder stays None)
3. connect refused -> graceful ARM: OFFLINE
4. MG400_VIZ_ARM=0 env override -> skip connect entirely (perf
   asserted: doesn't wait the timeout opener)
5. Env / config helpers (_arm_lifespan_enabled, _load_arm_endpoint_config)

These tests inject a fake ``open_connection`` into AsyncFeedbackStream
via class patch so they never touch real sockets.
"""

from __future__ import annotations

import asyncio
import os
import time
import unittest
from unittest.mock import patch

try:
    from fastapi.testclient import TestClient

    HAS_FASTAPI = True
except ImportError:
    HAS_FASTAPI = False


def _has_cv2() -> bool:
    try:
        import cv2  # noqa: F401
        import cv2.aruco  # noqa: F401

        return True
    except ImportError:
        return False


HAS_CV2 = _has_cv2()


def _make_feedback_frame_bytes() -> bytes:
    """Build one 1440-byte feedback frame with valid magic + known fields.

    Sets robot_mode=5 (ENABLE), tool_vector_actual=(230,0,60,-45,0,0)
    mm/deg, q_actual=(-0.01, 5.21, 32.4, -44.99, 0, 0) so the holder's
    resolved snapshot has values the test can assert on.
    """
    import numpy as np

    from robot_core.transport.feedback import (
        FEEDBACK_FRAME_SIZE,
        TEST_VALUE_MAGIC,
        FEEDBACK_DTYPE,
    )

    frame = np.zeros(1, dtype=FEEDBACK_DTYPE)
    # Note: the raw dtype uses CamelCase field names (EnableStatus / ErrorStatus);
    # parse_feedback() converts them to snake_case on the FeedbackFrame dataclass.
    frame[0]["test_value"] = TEST_VALUE_MAGIC
    frame[0]["robot_mode"] = 5
    frame[0]["EnableStatus"] = 1
    frame[0]["ErrorStatus"] = 0
    frame[0]["tool_vector_actual"] = [230.0, 0.0, 60.0, -45.0, 0.0, 0.0]
    frame[0]["q_actual"] = [-0.01, 5.21, 32.4, -44.99, 0.0, 0.0]
    raw = frame.tobytes()
    assert len(raw) == FEEDBACK_FRAME_SIZE
    return raw


class _FakeStreamReader:
    """asyncio.StreamReader stand-in that yields one feedback frame on repeat.

    ``readexactly`` MUST ``await`` something each call so the event loop
    can deliver cancellation -- otherwise the producer task busy-loops
    through fake frames and ``monitor.stop()`` hangs forever waiting for
    its cancel to be observed.
    """

    def __init__(self, payload: bytes):
        self._payload = payload
        self._buf = b""

    async def readexactly(self, n: int) -> bytes:
        # Yield control so cancellation can interrupt the read loop; tiny
        # sleep keeps test runs fast (a few hundred frames per millisecond)
        # while still allowing other tasks (e.g. monitor.stop) to run.
        await asyncio.sleep(0)
        while len(self._buf) < n:
            self._buf += self._payload
        out, self._buf = self._buf[:n], self._buf[n:]
        return out


class _FakeStreamWriter:
    def close(self):
        pass

    async def wait_closed(self):
        pass


def _make_fake_opener(payload: bytes):
    async def opener(host: str, port: int):
        return _FakeStreamReader(payload), _FakeStreamWriter()

    return opener


def _make_timeout_opener():
    async def opener(host: str, port: int):
        await asyncio.sleep(10)

    return opener


def _make_refused_opener():
    async def opener(host: str, port: int):
        raise ConnectionRefusedError(111, "Connection refused")

    return opener


def _patch_stream_opener(opener):
    """Class-level patch that injects ``opener`` into every AsyncFeedbackStream.

    AsyncFeedbackStream already supports the ``open_connection`` kwarg
    for tests; we hook __init__ so the lifespan-constructed stream
    automatically picks up our fake without create_app needing to know
    about it.
    """
    from robot_core.transport.feedback_stream import AsyncFeedbackStream

    original_init = AsyncFeedbackStream.__init__

    def patched_init(self, host, port, **kwargs):
        kwargs["open_connection"] = opener
        original_init(self, host, port, **kwargs)

    return patch.object(AsyncFeedbackStream, "__init__", patched_init)


def _build_app_with_fake_handeye(*, enable_arm_lifespan=True):
    """Build a viz app with a fake-camera handeye factory (no DmvSDK).

    The factory reads arm_state from ``app.state.arm_state_holder`` so
    it mirrors the production wiring: lifespan resolves the state,
    holder is the bridge, factory injects it per ws connection.
    """
    import cv2

    from robot_core.calibration.charuco import make_board
    from tests.test_handeye_session import _FakeCamera
    from tests.test_viz_workspace import _sample_bounds
    from viz.handeye_session import HandeyeSession
    from viz.server import create_app

    board = make_board()
    board_img = board.generateImage((800, 1100))
    synthetic_rgb = cv2.cvtColor(board_img, cv2.COLOR_GRAY2RGB)

    # The factory needs access to the app's holder. We can't reference
    # app until create_app returns. Build the factory as a closure over a
    # box, fill the box right after create_app.
    box: dict = {"holder": None}

    def factory():
        cam = _FakeCamera(frames_to_yield=[synthetic_rgb] * 50)
        arm_state = None if box["holder"] is None else box["holder"]["state"]
        session = HandeyeSession(
            camera=cam, board=board, target_views=5, arm_state=arm_state
        )
        return session, cam

    app = create_app(
        bounds=_sample_bounds(),
        grid_step_mm=50.0,
        handeye_session_factory=factory,
        enable_arm_lifespan=enable_arm_lifespan,
    )
    box["holder"] = app.state.arm_state_holder
    return app


@unittest.skipUnless(HAS_FASTAPI and HAS_CV2, "fastapi + opencv-contrib-python required")
class TestArmLifespanSuccess(unittest.IsolatedAsyncioTestCase):
    """Lifespan boots with a working fake stream -> holder filled.

    Drives the lifespan_context manually with asyncio instead of via
    TestClient. The previous TestClient approach was prone to hangs
    because the producer task in RobotStateMonitor loops forever
    reading from the fake stream, and cancellation interactions across
    sync/async boundaries inside Starlette's ws shutdown were brittle.
    Direct lifespan_context gives precise control over startup/shutdown.
    """

    def setUp(self):
        self._prev_env = os.environ.pop("MG400_VIZ_ARM", None)

    def tearDown(self):
        if self._prev_env is not None:
            os.environ["MG400_VIZ_ARM"] = self._prev_env

    async def test_holder_filled_during_lifespan_and_cleared_after(self):
        opener = _make_fake_opener(_make_feedback_frame_bytes())
        with _patch_stream_opener(opener):
            app = _build_app_with_fake_handeye()
            holder = app.state.arm_state_holder
            self.assertIsNone(holder["state"], "holder is None before lifespan starts")
            async with app.router.lifespan_context(app):
                self.assertIsNotNone(
                    holder["state"],
                    "lifespan should populate holder on connect success",
                )
                self.assertIsNotNone(holder["monitor"])
                self.assertIsNotNone(holder["stream"])
                # Give the consumer task time to process at least one frame
                # so the snapshot is populated and downstream code that
                # reads it (handeye factory) sees ARM: ONLINE values.
                for _ in range(20):
                    snap = holder["state"].snapshot
                    if snap is not None:
                        break
                    await asyncio.sleep(0.01)
                self.assertIsNotNone(snap, "snapshot should populate within 200ms")
                self.assertEqual(snap.robot_mode, 5)
                self.assertTrue(snap.is_enabled)
                self.assertFalse(snap.has_error)
                self.assertAlmostEqual(snap.tool_vector_actual[0], 230.0)
                self.assertAlmostEqual(snap.tool_vector_actual[3], -45.0)
            # After lifespan_context exits, shutdown ran -> holder cleared.
            self.assertIsNone(holder["state"], "shutdown should clear holder")
            self.assertIsNone(holder["monitor"])
            self.assertIsNone(holder["stream"])


@unittest.skipUnless(HAS_FASTAPI and HAS_CV2, "fastapi + opencv-contrib-python required")
class TestArmLifespanOffline(unittest.TestCase):
    """Failure paths: graceful degradation to ARM: OFFLINE (holder stays None)."""

    def setUp(self):
        self._prev_env = os.environ.pop("MG400_VIZ_ARM", None)

    def tearDown(self):
        if self._prev_env is not None:
            os.environ["MG400_VIZ_ARM"] = self._prev_env

    def test_connect_timeout_yields_offline(self):
        # Force a short connect_timeout so the test isn't slow.
        with patch(
            "viz.server._load_arm_endpoint_config",
            return_value={"ip": "10.255.255.1", "feedback_port": 30004, "connect_timeout_s": 0.1},
        ), _patch_stream_opener(_make_timeout_opener()):
            app = _build_app_with_fake_handeye()
            with TestClient(app):
                self.assertIsNone(app.state.arm_state_holder["state"])

    def test_connect_refused_yields_offline(self):
        with patch(
            "viz.server._load_arm_endpoint_config",
            return_value={"ip": "127.0.0.1", "feedback_port": 1, "connect_timeout_s": 0.5},
        ), _patch_stream_opener(_make_refused_opener()):
            app = _build_app_with_fake_handeye()
            with TestClient(app):
                self.assertIsNone(app.state.arm_state_holder["state"])

    def test_env_off_skips_connect_entirely(self):
        """MG400_VIZ_ARM=0 -> lifespan returns without trying to connect.

        Doubles as a perf assertion: the timeout opener would block for
        10s if the env-off branch failed to short-circuit; this test
        must finish in well under 1s.
        """
        os.environ["MG400_VIZ_ARM"] = "0"
        start = time.monotonic()
        with _patch_stream_opener(_make_timeout_opener()):
            app = _build_app_with_fake_handeye()
            with TestClient(app):
                self.assertIsNone(app.state.arm_state_holder["state"])
        elapsed = time.monotonic() - start
        self.assertLess(
            elapsed, 5.0, f"env-off path should not block on connect (took {elapsed:.1f}s)"
        )

    def test_create_app_flag_off_skips_connect_entirely(self):
        """enable_arm_lifespan=False (test default) -> no connect attempt."""
        with _patch_stream_opener(_make_timeout_opener()):
            app = _build_app_with_fake_handeye(enable_arm_lifespan=False)
            with TestClient(app):
                self.assertIsNone(app.state.arm_state_holder["state"])


@unittest.skipUnless(HAS_FASTAPI, "fastapi not installed")
class TestArmEnvAndConfigHelpers(unittest.TestCase):
    """Direct tests for _arm_lifespan_enabled / _load_arm_endpoint_config."""

    def setUp(self):
        self._prev_env = os.environ.pop("MG400_VIZ_ARM", None)

    def tearDown(self):
        if self._prev_env is not None:
            os.environ["MG400_VIZ_ARM"] = self._prev_env

    def test_default_enabled(self):
        from viz.server import _arm_lifespan_enabled

        self.assertTrue(_arm_lifespan_enabled())

    def test_off_variants_recognised(self):
        from viz.server import _arm_lifespan_enabled

        for v in ("0", "false", "no", "off", "FALSE", "Off"):
            os.environ["MG400_VIZ_ARM"] = v
            self.assertFalse(
                _arm_lifespan_enabled(),
                f"value {v!r} should disable arm lifespan",
            )

    def test_on_variants_recognised(self):
        from viz.server import _arm_lifespan_enabled

        for v in ("1", "true", "yes", "on"):
            os.environ["MG400_VIZ_ARM"] = v
            self.assertTrue(_arm_lifespan_enabled())

    def test_load_arm_endpoint_config_reads_robot_json(self):
        from viz.server import _load_arm_endpoint_config

        cfg = _load_arm_endpoint_config()
        self.assertEqual(cfg["ip"], "192.168.1.6")
        self.assertEqual(cfg["feedback_port"], 30004)
        self.assertGreater(cfg["connect_timeout_s"], 0)

    def test_env_overrides_apply(self):
        from viz.server import _load_arm_endpoint_config

        prev_ip = os.environ.pop("MG400_IP", None)
        prev_port = os.environ.pop("MG400_FEEDBACK_PORT", None)
        try:
            os.environ["MG400_IP"] = "10.0.0.42"
            os.environ["MG400_FEEDBACK_PORT"] = "31234"
            cfg = _load_arm_endpoint_config()
            self.assertEqual(cfg["ip"], "10.0.0.42")
            self.assertEqual(cfg["feedback_port"], 31234)
        finally:
            os.environ.pop("MG400_IP", None)
            os.environ.pop("MG400_FEEDBACK_PORT", None)
            if prev_ip is not None:
                os.environ["MG400_IP"] = prev_ip
            if prev_port is not None:
                os.environ["MG400_FEEDBACK_PORT"] = prev_port


if __name__ == "__main__":
    unittest.main()
