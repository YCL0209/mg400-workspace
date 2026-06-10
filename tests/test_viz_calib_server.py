"""ws round-trip tests for /ws/calib via fastapi.testclient.

Injects a fake camera + real CalibSession through ``calib_session_factory``
so the test exercises the full sender/receiver coroutines without needing
DmvSDK. Tests skip cleanly if fastapi or opencv-contrib-python is missing.
"""

import unittest

import numpy as np

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


@unittest.skipUnless(HAS_FASTAPI and HAS_CV2, "fastapi + opencv-contrib-python required")
class TestWsCalibEndpoint(unittest.TestCase):
    def setUp(self):
        import cv2

        from robot_core.calibration.charuco import make_board
        from viz.calib_session import CalibSession
        from viz.server import create_app
        from tests.test_calib_session import _FakeCamera
        from tests.test_viz_workspace import _sample_bounds

        board = make_board()
        board_img = board.generateImage((800, 1100))
        synthetic_rgb = cv2.cvtColor(board_img, cv2.COLOR_GRAY2RGB)

        def factory():
            cam = _FakeCamera(frames_to_yield=[synthetic_rgb] * 50)
            session = CalibSession(camera=cam, board=board, target_views=5)
            return session, cam

        self.app = create_app(
            bounds=_sample_bounds(),
            grid_step_mm=50.0,
            calib_session_factory=factory,
            enable_arm_lifespan=False,
        )
        self.client = TestClient(self.app)

    def test_first_message_is_calib_frame_with_detection(self):
        with self.client.websocket_connect("/ws/calib") as ws:
            msg = ws.receive_json()

        self.assertEqual(msg["type"], "calib_frame")
        self.assertIn("jpeg_b64", msg)
        self.assertIn("detection", msg)
        self.assertIn("captures", msg)
        # The synthetic frame IS the board, so detection should fire.
        self.assertTrue(msg["detection"]["board_visible"])

    def test_capture_action_advances_collected_count(self):
        with self.client.websocket_connect("/ws/calib") as ws:
            # Drain initial frame so the session has detected the board once.
            ws.receive_json()
            ws.send_text('{"action": "capture"}')
            # The next emitted frame should show captures.collected >= 1.
            # Stream a few frames to make sure we see the update reliably
            # (sender + receiver tasks race; capture takes 1 extra grab).
            seen_capture = False
            for _ in range(20):
                msg = ws.receive_json()
                if (
                    msg["type"] == "calib_frame"
                    and msg["captures"]["collected"] >= 1
                ):
                    seen_capture = True
                    break
            self.assertTrue(
                seen_capture, "capture action did not advance collected counter"
            )

    def test_solve_with_no_samples_returns_failure(self):
        with self.client.websocket_connect("/ws/calib") as ws:
            ws.receive_json()  # initial frame
            ws.send_text('{"action": "solve"}')
            for _ in range(10):
                msg = ws.receive_json()
                if msg.get("type") == "calib_result":
                    break
            self.assertEqual(msg["type"], "calib_result")
            self.assertFalse(msg["success"])
            self.assertIn("at least 3", msg["error"])


if __name__ == "__main__":
    unittest.main()
