"""Tests for the M0c-3 hand-eye artifact writer (write/read round-trip)."""

from __future__ import annotations

import json
import tempfile
import unittest
from datetime import datetime
from pathlib import Path

import numpy as np

from viz.handeye_artifact import write_artifact


class TestWriteHandeyeArtifactRoundTrip(unittest.TestCase):
    """Write, read back, verify schema matches PHASE2 §8.2.5."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp_path = Path(self._tmp.name) / "hand_eye.json"

    def tearDown(self):
        self._tmp.cleanup()

    def _write_sample(self, **overrides):
        defaults = dict(
            R=np.eye(3),
            t_mm=np.array([12.0, -3.5, 30.0]),
            rms_residual_mm=1.42,
            n_samples=18,
            method="CALIB_HAND_EYE_PARK",
            intrinsics_file="config/camera_intrinsics.json",
            intrinsics_rms_px=0.894,
            camera_serial="C1M6GM0W24460005",
            target_path=self.tmp_path,
        )
        defaults.update(overrides)
        return write_artifact(**defaults)

    def test_returns_target_path_and_file_exists(self):
        path = self._write_sample()
        self.assertEqual(path, self.tmp_path)
        self.assertTrue(path.exists())

    def test_payload_has_all_schema_fields(self):
        self._write_sample()
        data = json.loads(self.tmp_path.read_text(encoding="utf-8"))
        for key in (
            "T_tcp_cam",
            "frame",
            "method",
            "n_samples",
            "rms_residual_mm",
            "intrinsics_file",
            "intrinsics_rms_px",
            "camera_serial",
            "captured_at",
            "tool_version",
        ):
            self.assertIn(key, data, f"missing schema field: {key}")
        self.assertIn("R", data["T_tcp_cam"])
        self.assertIn("t", data["T_tcp_cam"])

    def test_R_is_3x3_nested_list_not_ndarray(self):
        self._write_sample()
        data = json.loads(self.tmp_path.read_text())
        R = data["T_tcp_cam"]["R"]
        self.assertIsInstance(R, list)
        self.assertEqual(len(R), 3)
        for row in R:
            self.assertIsInstance(row, list)
            self.assertEqual(len(row), 3)
        # Identity round-trips correctly.
        self.assertAlmostEqual(R[0][0], 1.0)
        self.assertAlmostEqual(R[1][1], 1.0)
        self.assertAlmostEqual(R[0][1], 0.0)

    def test_t_is_flat_3_vec_in_mm(self):
        self._write_sample()
        data = json.loads(self.tmp_path.read_text())
        t = data["T_tcp_cam"]["t"]
        self.assertIsInstance(t, list)
        self.assertEqual(len(t), 3)
        for v in t:
            self.assertIsInstance(v, float)
        self.assertAlmostEqual(t[0], 12.0)
        self.assertAlmostEqual(t[1], -3.5)
        self.assertAlmostEqual(t[2], 30.0)

    def test_frame_anchor_is_tcp(self):
        """PHASE2 §8.2 picks TCP anchoring (not flange)."""
        self._write_sample()
        data = json.loads(self.tmp_path.read_text())
        self.assertEqual(data["frame"], "tcp")

    def test_method_passthrough(self):
        self._write_sample(method="CALIB_HAND_EYE_TSAI")
        data = json.loads(self.tmp_path.read_text())
        self.assertEqual(data["method"], "CALIB_HAND_EYE_TSAI")

    def test_intrinsics_cross_record(self):
        """intrinsics_file + rms tell readers WHICH K this T was solved against."""
        self._write_sample(
            intrinsics_file="config/camera_intrinsics.json",
            intrinsics_rms_px=0.42,
        )
        data = json.loads(self.tmp_path.read_text())
        self.assertEqual(data["intrinsics_file"], "config/camera_intrinsics.json")
        self.assertAlmostEqual(data["intrinsics_rms_px"], 0.42)

    def test_intrinsics_rms_px_none_allowed(self):
        """intrinsics file may not record rms (older M0b runs); None is fine."""
        self._write_sample(intrinsics_rms_px=None)
        data = json.loads(self.tmp_path.read_text())
        self.assertIsNone(data["intrinsics_rms_px"])

    def test_camera_serial_none_allowed(self):
        self._write_sample(camera_serial=None)
        data = json.loads(self.tmp_path.read_text())
        self.assertIsNone(data["camera_serial"])

    def test_captured_at_is_iso8601(self):
        self._write_sample()
        data = json.loads(self.tmp_path.read_text())
        # Should parse back; raises ValueError otherwise.
        self.assertIsNotNone(datetime.fromisoformat(data["captured_at"]))

    def test_tool_version_present(self):
        self._write_sample()
        data = json.loads(self.tmp_path.read_text())
        self.assertTrue(data["tool_version"].startswith("M0c"))

    def test_overwrites_existing_artifact(self):
        self._write_sample(rms_residual_mm=4.5)
        self._write_sample(rms_residual_mm=1.1)
        data = json.loads(self.tmp_path.read_text())
        self.assertAlmostEqual(data["rms_residual_mm"], 1.1)

    def test_creates_parent_directory_if_missing(self):
        nested = Path(self._tmp.name) / "nested" / "subdir" / "hand_eye.json"
        self._write_sample(target_path=nested)
        self.assertTrue(nested.exists())

    def test_rejects_bad_R_shape(self):
        with self.assertRaises(ValueError) as ctx:
            self._write_sample(R=np.eye(4))
        self.assertIn("3x3", str(ctx.exception))

    def test_rejects_bad_t_length(self):
        with self.assertRaises(ValueError) as ctx:
            self._write_sample(t_mm=np.array([1.0, 2.0]))
        self.assertIn("3-vec", str(ctx.exception))

    def test_nan_in_rms_residual_raises_at_write(self):
        """Finding 27 contract: NaN must not slip into the artifact."""
        with self.assertRaises(ValueError):
            self._write_sample(rms_residual_mm=float("nan"))
        # And no file should have been written.
        self.assertFalse(self.tmp_path.exists())


if __name__ == "__main__":
    unittest.main()
