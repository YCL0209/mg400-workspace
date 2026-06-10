"""ws round-trip tests for /ws/handeye via fastapi.testclient.

Injects a fake camera + real HandeyeSession (arm offline) through
``handeye_session_factory`` so the test exercises the full sender/
receiver coroutines without DmvSDK or a live MG400.
"""

import unittest


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
class TestWsHandeyeEndpoint(unittest.TestCase):
    def setUp(self):
        import cv2

        from robot_core.calibration.charuco import make_board
        from tests.test_handeye_session import _FakeArmSnapshot, _FakeArmState, _FakeCamera
        from tests.test_viz_workspace import _sample_bounds
        from viz.handeye_session import HandeyeSession
        from viz.server import create_app

        board = make_board()
        board_img = board.generateImage((800, 1100))
        synthetic_rgb = cv2.cvtColor(board_img, cv2.COLOR_GRAY2RGB)

        # Default factory ships with a live arm_state so we can also assert
        # the arm payload round-trips. M0c-1 production wires None here --
        # tests prove BOTH paths.
        self._arm = _FakeArmState(snapshot=_FakeArmSnapshot())

        def factory():
            cam = _FakeCamera(frames_to_yield=[synthetic_rgb] * 50)
            session = HandeyeSession(
                camera=cam, board=board, target_views=5, arm_state=self._arm
            )
            return session, cam

        self.app = create_app(
            bounds=_sample_bounds(),
            grid_step_mm=50.0,
            handeye_session_factory=factory,
            enable_arm_lifespan=False,
        )
        self.client = TestClient(self.app)

    def test_first_message_is_handeye_frame_with_arm_payload(self):
        with self.client.websocket_connect("/ws/handeye") as ws:
            msg = ws.receive_json()

        self.assertEqual(msg["type"], "handeye_frame")
        self.assertIn("jpeg_b64", msg)
        self.assertIn("detection", msg)
        self.assertIn("captures", msg)
        self.assertIn("arm", msg)
        # Arm injected, so the wire payload should show ONLINE.
        self.assertTrue(msg["arm"]["available"])
        self.assertEqual(msg["arm"]["mode"], 5)
        # Detection should fire since the synthetic frame IS the board.
        self.assertTrue(msg["detection"]["board_visible"])

    def test_capture_action_advances_collected_count(self):
        with self.client.websocket_connect("/ws/handeye") as ws:
            ws.receive_json()
            ws.send_text('{"action": "capture"}')
            seen_capture = False
            for _ in range(20):
                msg = ws.receive_json()
                if (
                    msg["type"] == "handeye_frame"
                    and msg["captures"]["collected"] >= 1
                ):
                    seen_capture = True
                    break
            self.assertTrue(
                seen_capture, "capture action did not advance collected counter"
            )

    def test_solve_action_with_no_samples_returns_failure(self):
        """Solve without any captured samples: at-least-3 failure."""
        with self.client.websocket_connect("/ws/handeye") as ws:
            ws.receive_json()
            ws.send_text('{"action": "solve"}')
            for _ in range(10):
                msg = ws.receive_json()
                if msg.get("type") == "handeye_result":
                    break
            self.assertEqual(msg["type"], "handeye_result")
            self.assertFalse(msg["success"])
            self.assertIn("at least 3", msg["error"])
            self.assertEqual(msg["n_samples"], 0)


if __name__ == "__main__":
    unittest.main()
