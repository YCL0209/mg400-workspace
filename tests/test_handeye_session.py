"""Tests for the M0c-1 hand-eye calibration session skeleton.

Mirrors test_calib_session.py: synthetic-board frame through a fake
camera, no DmvSDK needed; cv2.aruco itself is required (skips if
opencv-contrib-python is absent).

M0c-1 specifics covered here:

- arm.available toggles correctly based on arm_state hook
- captured HandeyeSample carries arm_pose (or None when offline)
- solve() returns the M0c-1 stub error, doesn't touch disk
- HandeyeFrameMessage / HandeyeResultMessage are JSON-clean (no NaN,
  finding 27 lesson)
"""

import asyncio
import base64
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np


def _has_cv2() -> bool:
    try:
        import cv2  # noqa: F401
        import cv2.aruco  # noqa: F401

        return True
    except ImportError:
        return False


HAS_CV2 = _has_cv2()


class _FakeCamera:
    """Stand-in for DeltaCamera; yields canned frames + tracks open/close."""

    def __init__(self, frames_to_yield):
        self._frames = list(frames_to_yield)
        self.is_open = True

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def start_continuous(self):
        pass

    def stop_continuous(self):
        pass

    def grab_continuous_rgb(self, timeout_ms=1000):
        if not self._frames:
            return None
        return self._frames.pop(0)


class _FakeArmSnapshot:
    """Stands in for RobotStateSnapshot in unit tests.

    Matches the duck-typed fields HandeyeSession._snapshot_arm /
    _arm_payload read: tool_vector_actual (6-tuple), joints (4-tuple via
    property), robot_mode (int), is_enabled / has_error (bools).
    """

    def __init__(
        self,
        *,
        tool_vector_actual=(230.0, 0.0, 60.0, -45.0, 0.0, 0.0),
        q_actual=(-0.01, 5.21, 32.4, -44.99),
        robot_mode=5,
        is_enabled=True,
        has_error=False,
    ):
        self.tool_vector_actual = tool_vector_actual
        self._q_actual = q_actual
        self.robot_mode = robot_mode
        self.is_enabled = is_enabled
        self.has_error = has_error

    @property
    def joints(self):
        return self._q_actual


class _FakeArmState:
    """Stands in for RobotState; exposes a .snapshot property."""

    def __init__(self, snapshot=None):
        self._snapshot = snapshot

    @property
    def snapshot(self):
        return self._snapshot

    def set_snapshot(self, snapshot):
        self._snapshot = snapshot


@unittest.skipUnless(HAS_CV2, "opencv-contrib-python not installed")
class TestHandeyeSession(unittest.TestCase):
    """End-to-end-ish: feed synthetic frames + verify outputs (arm offline)."""

    def setUp(self):
        import cv2

        from robot_core.calibration.charuco import make_board
        from viz.handeye_session import HandeyeSession

        self.board = make_board()
        board_img = self.board.generateImage((800, 1100))
        self.synthetic_rgb = cv2.cvtColor(board_img, cv2.COLOR_GRAY2RGB)

        self.fake_camera = _FakeCamera(frames_to_yield=[self.synthetic_rgb] * 50)
        self._tmp = tempfile.TemporaryDirectory()
        self.session = HandeyeSession(
            camera=self.fake_camera,
            board=self.board,
            target_views=5,
            camera_serial="TEST-SN-001",
            artifact_path=Path(self._tmp.name) / "hand_eye.json",
        )

    def tearDown(self):
        self._tmp.cleanup()

    def _run(self, coro):
        # Fresh loop per call -- robust under unittest discover ordering
        # (some earlier test in alphabetical order can close the main-thread
        # loop, and Python 3.12 deprecates get_event_loop() auto-creating one).
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(coro)
        finally:
            loop.close()

    def test_stream_frame_emits_handeye_frame_with_jpeg(self):
        msg = self._run(self.session.stream_frame())
        self.assertEqual(msg["type"], "handeye_frame")
        self.assertIn("jpeg_b64", msg)
        self.assertGreater(len(msg["jpeg_b64"]), 100)
        raw = base64.b64decode(msg["jpeg_b64"])
        self.assertEqual(raw[:3], b"\xff\xd8\xff")

    def test_detection_reports_board_visible(self):
        msg = self._run(self.session.stream_frame())
        det = msg["detection"]
        self.assertTrue(det["board_visible"])
        self.assertGreater(len(det["marker_ids"]), 0)
        # Same 7x10 board as the calib path -> 54 corners total.
        self.assertEqual(det["charuco_corners_total"], 6 * 9)
        self.assertGreater(det["charuco_corners_found"], 30)

    def test_arm_available_false_when_no_arm_state(self):
        """No arm_state hook -> arm.available=False, no pose/joints leak."""
        msg = self._run(self.session.stream_frame())
        arm = msg["arm"]
        self.assertEqual(arm["available"], False)
        # available=False means everything else is intentionally omitted
        # (TypedDict total=False). Frontend reads only `available` first.
        self.assertNotIn("pose", arm)
        self.assertNotIn("joints", arm)

    def test_capture_when_arm_offline_records_none_pose(self):
        """Sample buffer keeps the entry but arm_pose / arm_joints are None."""
        self._run(self.session.stream_frame())
        self.session.apply_action({"action": "capture"})
        self.assertEqual(self.session.collected, 1)
        sample = self.session._samples[0]
        self.assertIsNone(sample.arm_pose)
        self.assertIsNone(sample.arm_joints)

    def test_discard_pops_last_sample(self):
        self._run(self.session.stream_frame())
        self.session.apply_action({"action": "capture"})
        self.session.apply_action({"action": "capture"})
        self.assertEqual(self.session.collected, 2)
        self.session.apply_action({"action": "discard"})
        self.assertEqual(self.session.collected, 1)

    def test_reset_clears_buffer(self):
        self._run(self.session.stream_frame())
        self.session.apply_action({"action": "capture"})
        self.session.apply_action({"action": "capture"})
        self.session.apply_action({"action": "reset"})
        self.assertEqual(self.session.collected, 0)

    def test_solve_fails_when_no_samples_with_arm_pose(self):
        """All offline captures -> 0 valid samples -> need-at-least-3 error."""
        self._run(self.session.stream_frame())
        # Capture a few times but arm is offline -> all sample.arm_pose=None.
        for _ in range(5):
            self.session.apply_action({"action": "capture"})

        result = self.session.apply_action({"action": "solve"})
        self.assertEqual(result["type"], "handeye_result")
        self.assertFalse(result["success"])
        self.assertEqual(result["n_samples"], 0)
        self.assertEqual(result["n_samples_dropped"], 5)
        self.assertIn("at least 3", result["error"])
        # NaN-omission contract: failure paths must not carry numeric rms.
        self.assertNotIn("rms_residual_mm", result)
        self.assertFalse(self.session.artifact_path.exists())

    def test_solve_failure_result_is_json_clean(self):
        """Failure-path schema must serialise without NaN (finding 27)."""
        result = self.session.apply_action({"action": "solve"})
        try:
            json.dumps(result, allow_nan=False)
        except ValueError as e:
            self.fail(f"handeye_result not JSON-clean on failure: {e}")

    def test_unknown_action_is_logged_and_ignored(self):
        self.assertIsNone(self.session.apply_action({"action": "nuke"}))


@unittest.skipUnless(HAS_CV2, "opencv-contrib-python not installed")
class TestHandeyeSessionWithArm(unittest.TestCase):
    """Same session, but with a mocked arm_state plugged in (M0c-2 preview)."""

    def setUp(self):
        import cv2

        from robot_core.calibration.charuco import make_board
        from viz.handeye_session import HandeyeSession

        self.board = make_board()
        board_img = self.board.generateImage((800, 1100))
        self.synthetic_rgb = cv2.cvtColor(board_img, cv2.COLOR_GRAY2RGB)
        self.snap = _FakeArmSnapshot()
        self.arm_state = _FakeArmState(snapshot=self.snap)

        self.session = HandeyeSession(
            camera=_FakeCamera(frames_to_yield=[self.synthetic_rgb] * 50),
            arm_state=self.arm_state,
            board=self.board,
            target_views=5,
        )

    def _run(self, coro):
        # Fresh loop per call -- robust under unittest discover ordering
        # (some earlier test in alphabetical order can close the main-thread
        # loop, and Python 3.12 deprecates get_event_loop() auto-creating one).
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(coro)
        finally:
            loop.close()

    def test_arm_payload_reports_online_with_pose_and_joints(self):
        msg = self._run(self.session.stream_frame())
        arm = msg["arm"]
        self.assertTrue(arm["available"])
        # Pose comes from tool_vector_actual[:4] -- finding 1 + B6.
        self.assertAlmostEqual(arm["pose"]["x"], 230.0)
        self.assertAlmostEqual(arm["pose"]["y"], 0.0)
        self.assertAlmostEqual(arm["pose"]["z"], 60.0)
        self.assertAlmostEqual(arm["pose"]["r"], -45.0)
        self.assertEqual(len(arm["joints"]), 4)
        self.assertEqual(arm["mode"], 5)
        self.assertTrue(arm["enabled"])
        self.assertFalse(arm["has_error"])

    def test_capture_with_arm_online_records_pose_and_joints(self):
        self._run(self.session.stream_frame())
        self.session.apply_action({"action": "capture"})
        sample = self.session._samples[0]
        self.assertIsNotNone(sample.arm_pose)
        self.assertEqual(len(sample.arm_pose), 4)
        self.assertAlmostEqual(sample.arm_pose[0], 230.0)
        self.assertIsNotNone(sample.arm_joints)
        self.assertEqual(len(sample.arm_joints), 4)

    def test_arm_payload_offline_when_snapshot_becomes_none(self):
        """RobotState before first feedback frame: snapshot=None, treat as OFFLINE."""
        self.arm_state.set_snapshot(None)
        msg = self._run(self.session.stream_frame())
        self.assertEqual(msg["arm"]["available"], False)

    def test_arm_payload_offline_when_snapshot_missing_fields(self):
        """Hand-eye shouldn't crash if the duck doesn't quack -- degrade to OFFLINE."""

        class _BadSnapshot:
            # missing tool_vector_actual, joints, robot_mode
            is_enabled = True
            has_error = False

        self.arm_state.set_snapshot(_BadSnapshot())
        msg = self._run(self.session.stream_frame())
        self.assertEqual(msg["arm"]["available"], False)

    def test_frame_message_json_serialises_with_arm_payload(self):
        msg = self._run(self.session.stream_frame())
        try:
            json.dumps(msg, allow_nan=False)
        except (ValueError, TypeError) as e:
            self.fail(f"handeye_frame not JSON-clean with arm online: {e}")


@unittest.skipUnless(HAS_CV2, "opencv-contrib-python not installed")
class TestPoseToMatrix(unittest.TestCase):
    """Direct tests for the (x,y,z,r) -> T_base<-tcp helper.

    Convention is load-bearing for the whole solver: if r maps to the
    wrong axis or the units are wrong, every downstream sample is
    miscomputed. Pin it explicitly.
    """

    def _M(self, x, y, z, r):
        from viz.handeye_session import HandeyeSession

        return HandeyeSession._pose_to_matrix(x, y, z, r)

    def test_zero_pose_is_identity_with_zero_translation(self):
        T = self._M(0, 0, 0, 0)
        np.testing.assert_allclose(T, np.eye(4), atol=1e-12)

    def test_translation_is_in_metres(self):
        """1000mm input -> 1.0 metre in T. Matches cv2's metre convention."""
        T = self._M(1000.0, 2000.0, -500.0, 0)
        np.testing.assert_allclose(T[:3, 3], [1.0, 2.0, -0.5], atol=1e-12)

    def test_r_is_yaw_about_z(self):
        """r=90deg -> +x in TCP becomes +y in base (right-hand Rz)."""
        T = self._M(0, 0, 0, 90)
        x_in_tcp = np.array([1.0, 0.0, 0.0, 1.0])
        # T @ x_tcp = x_base
        x_in_base = T @ x_in_tcp
        np.testing.assert_allclose(x_in_base[:3], [0.0, 1.0, 0.0], atol=1e-12)

    def test_negative_yaw_is_consistent(self):
        T_pos = self._M(0, 0, 0, 45)
        T_neg = self._M(0, 0, 0, -45)
        # Rz(45) @ Rz(-45) == identity
        np.testing.assert_allclose(T_pos @ T_neg, np.eye(4), atol=1e-12)


@unittest.skipUnless(HAS_CV2, "opencv-contrib-python not installed")
class TestHandeyeSolverFailurePaths(unittest.TestCase):
    """Solver should fail loudly + cleanly when prereqs aren't met."""

    def setUp(self):
        import cv2

        from robot_core.calibration.charuco import make_board
        from viz.handeye_session import HandeyeSession

        self.board = make_board()
        board_img = self.board.generateImage((800, 1100))
        self.synthetic_rgb = cv2.cvtColor(board_img, cv2.COLOR_GRAY2RGB)
        self._tmp = tempfile.TemporaryDirectory()
        self.session = HandeyeSession(
            camera=_FakeCamera(frames_to_yield=[self.synthetic_rgb] * 50),
            arm_state=_FakeArmState(snapshot=_FakeArmSnapshot()),
            board=self.board,
            target_views=5,
            # Intrinsics injected so PnP step CAN run -- failure tests
            # below selectively unset this to drive the "no K" path.
            intrinsics_K=np.array(
                [[800.0, 0.0, 400.0], [0.0, 800.0, 550.0], [0.0, 0.0, 1.0]]
            ),
            intrinsics_dist=np.zeros(5),
            artifact_path=Path(self._tmp.name) / "hand_eye.json",
        )

    def tearDown(self):
        self._tmp.cleanup()

    def _run(self, coro):
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(coro)
        finally:
            loop.close()

    def test_fewer_than_three_paired_samples_fails(self):
        self._run(self.session.stream_frame())
        self.session.apply_action({"action": "capture"})
        self.session.apply_action({"action": "capture"})  # only 2 paired
        result = self.session.apply_action({"action": "solve"})
        self.assertFalse(result["success"])
        self.assertEqual(result["n_samples"], 2)
        self.assertIn("at least 3", result["error"])
        self.assertNotIn("rms_residual_mm", result)
        self.assertFalse(self.session.artifact_path.exists())

    def test_intrinsics_missing_fails(self):
        # Strip K to drive the "no K" path; need 3+ paired samples first
        # so we don't trip the earlier guard.
        self._run(self.session.stream_frame())
        for _ in range(3):
            self.session.apply_action({"action": "capture"})
        self.session._K = None
        result = self.session.apply_action({"action": "solve"})
        self.assertFalse(result["success"])
        self.assertIn("intrinsics not loaded", result["error"])
        self.assertNotIn("rms_residual_mm", result)
        self.assertFalse(self.session.artifact_path.exists())


@unittest.skipUnless(HAS_CV2, "opencv-contrib-python not installed")
class TestHandeyeSolverSyntheticRoundTrip(unittest.TestCase):
    """End-to-end correctness pin: known T_tcp<-cam should be recovered.

    Strategy:
    1. Pick a known T_tcp_cam (rotation + translation in metres).
    2. Generate N varied T_base_tcp poses.
    3. Pick an arbitrary T_base_board ("the board sits here in base").
    4. For each i, compute T_cam_board = inv(T_tcp_cam) @ inv(T_base_tcp_i)
       @ T_base_board. This is the PnP result the real cam WOULD have
       observed if the geometry was exact.
    5. Build HandeyeSamples whose arm_pose encodes T_base_tcp and whose
       (corners, ids) trigger a mocked estimatePoseCharucoBoard returning
       the right (rvec, tvec).
    6. Run solve() with a mocked cv2.calibrateHandEye -- skip the
       mocking for cv2.calibrateHandEye itself; the real implementation
       should recover T_tcp_cam from this clean data.
    7. Verify recovered R/t close to the known one + artifact written.

    This is the test that catches axis-convention errors (Rz direction,
    metre/mm swaps, gripper2base vs base2gripper mixups) BEFORE we hit
    hardware in M0c-2.
    """

    def setUp(self):
        import cv2

        from robot_core.calibration.charuco import make_board
        from viz.handeye_session import HandeyeSample, HandeyeSession

        self.cv2 = cv2
        self.board = make_board()
        board_img = self.board.generateImage((800, 1100))
        self.synthetic_rgb = cv2.cvtColor(board_img, cv2.COLOR_GRAY2RGB)
        self._tmp = tempfile.TemporaryDirectory()
        self.session = HandeyeSession(
            camera=_FakeCamera(frames_to_yield=[self.synthetic_rgb] * 5),
            board=self.board,
            target_views=10,
            intrinsics_K=np.array(
                [[800.0, 0.0, 400.0], [0.0, 800.0, 550.0], [0.0, 0.0, 1.0]]
            ),
            intrinsics_dist=np.zeros(5),
            artifact_path=Path(self._tmp.name) / "hand_eye.json",
        )
        self._HandeyeSample = HandeyeSample

    def tearDown(self):
        self._tmp.cleanup()

    @staticmethod
    def _rz(deg):
        r = math.radians(deg)
        c, s = math.cos(r), math.sin(r)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)

    @staticmethod
    def _T(R, t):
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    @staticmethod
    def _rx(deg):
        r = math.radians(deg)
        c, s = math.cos(r), math.sin(r)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)

    @staticmethod
    def _ry(deg):
        r = math.radians(deg)
        c, s = math.cos(r), math.sin(r)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)

    def _diverse_gripper_poses(self):
        """Yield (R_base_tcp, t_base_tcp_m) pairs with rotation-axis diversity.

        Real MG400 is yaw-only (degenerate for hand-eye rotation
        recovery). To validate algorithm WIRING -- units, gripper2base
        vs base2gripper convention, residual computation -- we bypass
        _pose_to_matrix in this test and feed diverse rotations.

        M0c-2 real-arm validation will face the genuine MG400 degeneracy
        (translation recoverable, rotation partial); the algorithm
        correctness shown here is a prerequisite for interpreting those
        results.
        """
        return [
            (self._rz(0), np.array([0.20, 0.00, 0.10])),
            (self._rz(30) @ self._rx(15), np.array([0.20, 0.05, 0.10])),
            (self._rz(-30) @ self._ry(15), np.array([0.25, -0.05, 0.10])),
            (self._rx(45), np.array([0.18, 0.08, 0.12])),
            (self._ry(-30) @ self._rz(20), np.array([0.22, -0.03, 0.08])),
            (self._rx(-15) @ self._rz(45), np.array([0.26, 0.02, 0.11])),
        ]

    def _build_samples_and_pnp_returns(self, T_tcp_cam_known, T_base_board, poses):
        """Generate (samples, [(rvec, tvec)]) for a given known geometry."""
        T_tcp_cam_inv = np.linalg.inv(T_tcp_cam_known)
        rvecs_tvecs = []
        T_base_tcps = []
        samples = []
        for (R_bt, t_bt) in poses:
            T_base_tcp = self._T(R_bt, t_bt)
            T_cam_board = T_tcp_cam_inv @ np.linalg.inv(T_base_tcp) @ T_base_board
            rvec, _ = self.cv2.Rodrigues(T_cam_board[:3, :3])
            tvec = T_cam_board[:3, 3].reshape(3, 1)
            rvecs_tvecs.append((rvec, tvec))
            T_base_tcps.append(T_base_tcp)
            samples.append(
                self._HandeyeSample(
                    corners=np.zeros((4, 1, 2), dtype=np.float32),
                    ids=np.zeros((4, 1), dtype=np.int32),
                    image_size=(1080, 1440),
                    # arm_pose is irrelevant once _pose_to_matrix is patched
                    # to return T_base_tcps[i] directly; we still set non-None
                    # so _select_valid_samples keeps the sample.
                    arm_pose=(0.0, 0.0, 0.0, 0.0),
                    arm_joints=(0.0, 0.0, 0.0, 0.0),
                    captured_at_monotonic=0.0,
                )
            )
        return samples, rvecs_tvecs, T_base_tcps

    def test_synthetic_round_trip_recovers_known_T_tcp_cam(self):
        from unittest.mock import patch

        # Known answer: camera mounted with a 90deg pitch + 50mm offset.
        # Realistic eye-in-hand mounting points camera Z forward; we pick
        # this to break the Rz-only degeneracy in the synthetic data.
        R_known = self._rx(90) @ self._rz(30)
        t_known_m = np.array([0.05, -0.02, 0.10])
        T_tcp_cam_known = self._T(R_known, t_known_m)
        T_base_board = self._T(np.eye(3), np.array([0.30, 0.10, 0.0]))

        poses = self._diverse_gripper_poses()
        samples, rvecs_tvecs, T_base_tcps = self._build_samples_and_pnp_returns(
            T_tcp_cam_known, T_base_board, poses
        )
        self.session._samples = samples

        call_idx = {"i": 0}

        def fake_estimate(*args, **kwargs):
            i = call_idx["i"]
            call_idx["i"] += 1
            rvec, tvec = rvecs_tvecs[i]
            return True, rvec, tvec

        # Patch _pose_to_matrix to return our diverse-axis T_base_tcps in
        # order. arm_pose tuples are irrelevant; we just need the sample
        # iteration to map index -> our matrix.
        seq = {"i": 0}

        def fake_pose_to_matrix(x, y, z, r):
            T = T_base_tcps[seq["i"]]
            seq["i"] += 1
            return T

        with patch(
            "viz.handeye_session.aruco.estimatePoseCharucoBoard",
            side_effect=fake_estimate,
        ), patch.object(
            type(self.session), "_pose_to_matrix", staticmethod(fake_pose_to_matrix)
        ):
            result = self.session.apply_action({"action": "solve"})

        self.assertTrue(
            result["success"], f"solve failed: {result.get('error', '?')}"
        )
        self.assertEqual(result["n_samples"], 6)
        self.assertEqual(result["n_samples_dropped"], 0)
        self.assertEqual(result["method"], "CALIB_HAND_EYE_PARK")

        R_recovered = np.asarray(result["R"])
        t_recovered_mm = np.asarray(result["t"])
        # Clean data + diverse axes -> sub-micrometre recovery.
        np.testing.assert_allclose(R_recovered, R_known, atol=1e-6)
        np.testing.assert_allclose(t_recovered_mm, t_known_m * 1000.0, atol=1e-3)
        self.assertLess(result["rms_residual_mm"], 1e-3)
        self.assertTrue(self.session.artifact_path.exists())

    def test_success_result_is_json_clean(self):
        """Reuse synthetic round-trip and assert JSON output never has NaN."""
        from unittest.mock import patch

        R_known = self._rx(90)
        t_known_m = np.array([0.0, 0.0, 0.05])
        T_tcp_cam_known = self._T(R_known, t_known_m)
        T_base_board = self._T(np.eye(3), np.array([0.25, 0.0, 0.0]))

        poses = self._diverse_gripper_poses()[:4]
        samples, rvecs_tvecs, T_base_tcps = self._build_samples_and_pnp_returns(
            T_tcp_cam_known, T_base_board, poses
        )
        self.session._samples = samples

        call_idx = {"i": 0}

        def fake_estimate(*args, **kwargs):
            i = call_idx["i"]
            call_idx["i"] += 1
            rvec, tvec = rvecs_tvecs[i]
            return True, rvec, tvec

        seq = {"i": 0}

        def fake_pose_to_matrix(x, y, z, r):
            T = T_base_tcps[seq["i"]]
            seq["i"] += 1
            return T

        with patch(
            "viz.handeye_session.aruco.estimatePoseCharucoBoard",
            side_effect=fake_estimate,
        ), patch.object(
            type(self.session), "_pose_to_matrix", staticmethod(fake_pose_to_matrix)
        ):
            result = self.session.apply_action({"action": "solve"})

        self.assertTrue(result["success"])
        try:
            json.dumps(result, allow_nan=False)
        except (ValueError, TypeError) as e:
            self.fail(f"handeye_result not JSON-clean on success: {e}")


if __name__ == "__main__":
    unittest.main()
