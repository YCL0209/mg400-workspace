"""Smoke-test the FastAPI ws server: client connects → workspace message arrives.

Uses ``fastapi.testclient.TestClient`` (Starlette's sync ws helper). The actual
runtime is async, but the test harness wraps it — we just receive the JSON and
assert its shape. Skips cleanly if FastAPI isn't installed.
"""

import unittest

try:
    from fastapi.testclient import TestClient

    HAS_FASTAPI = True
except ImportError:  # pragma: no cover — only when deps missing
    HAS_FASTAPI = False

from tests.test_viz_workspace import _sample_bounds


@unittest.skipUnless(HAS_FASTAPI, "fastapi not installed")
class TestVizWsEndpoint(unittest.TestCase):
    """``GET /ws`` (upgraded) sends one workspace message on accept."""

    def setUp(self):
        from viz.server import create_app

        self.app = create_app(bounds=_sample_bounds(), grid_step_mm=50.0)
        self.client = TestClient(self.app)

    def test_ws_emits_workspace_on_connect(self):
        with self.client.websocket_connect("/ws") as ws:
            msg = ws.receive_json()

        self.assertEqual(msg["type"], "workspace")
        # Core geometry forwarded from the sample bounds we injected.
        self.assertAlmostEqual(msg["annulus_inner_mm"], 123.83)
        self.assertAlmostEqual(msg["annulus_outer_mm"], 440.0)
        self.assertEqual(msg["grid_step_mm"], 50.0)

    def test_ws_holds_connection_open_after_workspace(self):
        """M1 keeps the socket alive past the initial push (M2 will reuse it)."""
        with self.client.websocket_connect("/ws") as ws:
            ws.receive_json()  # workspace
            # Send a no-op client message; server should accept without closing.
            ws.send_text("ping")
            # If the server had closed, the context manager exit would raise.
        # Reaching here = clean disconnect on context exit.


if __name__ == "__main__":
    unittest.main()
