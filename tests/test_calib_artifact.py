"""Tests for the M0b calibration artifact writer (write/read round-trip).

Uses a temp directory so we don't touch the real ``config/camera_intrinsics.json``.
cv2-dependent paths skip cleanly when opencv-contrib-python is absent (the
artifact writer itself doesn't import cv2 but the CHARUCO_BOARD fixture does).
"""

from __future__ import annotations

import json
import tempfile
import unittest
from datetime import datetime
from pathlib import Path

import numpy as np

from robot_core.calibration.charuco import CHARUCO_BOARD
from viz.calib_artifact import write_artifact


class TestWriteArtifactRoundTrip(unittest.TestCase):
    """Write artifact, read back, verify schema matches PHASE2 §8.1.4."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp_path = Path(self._tmp.name) / "camera_intrinsics.json"

    def tearDown(self):
        self._tmp.cleanup()

    def _write_sample(self, **overrides):
        defaults = dict(
            K=np.array([[800.0, 0.0, 640.0], [0.0, 800.0, 360.0], [0.0, 0.0, 1.0]]),
            dist=np.array([[0.1], [-0.05], [0.001], [0.002], [0.0]]),
            rms_px=0.42,
            image_size=(1440, 1080),
            n_views=23,
            board_spec=CHARUCO_BOARD,
            camera_serial="C1M6GM0W24460005",
            target_path=self.tmp_path,
        )
        defaults.update(overrides)
        return write_artifact(**defaults)

    def test_returns_target_path(self):
        path = self._write_sample()
        self.assertEqual(path, self.tmp_path)
        self.assertTrue(path.exists())

    def test_payload_is_valid_json_with_all_schema_fields(self):
        self._write_sample()
        data = json.loads(self.tmp_path.read_text(encoding="utf-8"))
        for key in (
            "K",
            "dist",
            "image_width",
            "image_height",
            "rms_px",
            "n_views",
            "board",
            "camera_serial",
            "captured_at",
            "tool_version",
        ):
            self.assertIn(key, data, f"missing schema field: {key}")

    def test_K_is_3x3_nested_list_not_ndarray(self):
        """Operators read this JSON; ndarray repr would be unreadable."""
        self._write_sample()
        data = json.loads(self.tmp_path.read_text())
        self.assertIsInstance(data["K"], list)
        self.assertEqual(len(data["K"]), 3)
        for row in data["K"]:
            self.assertIsInstance(row, list)
            self.assertEqual(len(row), 3)
        # Sanity check values made the round trip.
        self.assertAlmostEqual(data["K"][0][0], 800.0)
        self.assertAlmostEqual(data["K"][1][2], 360.0)

    def test_dist_is_flat_list(self):
        """cv2 returns dist as Nx1 column; we flatten so JSON is concise."""
        self._write_sample()
        data = json.loads(self.tmp_path.read_text())
        self.assertIsInstance(data["dist"], list)
        for v in data["dist"]:
            self.assertIsInstance(v, float)
        self.assertEqual(len(data["dist"]), 5)

    def test_image_size_split_to_width_height(self):
        self._write_sample(image_size=(1440, 1080))
        data = json.loads(self.tmp_path.read_text())
        self.assertEqual(data["image_width"], 1440)
        self.assertEqual(data["image_height"], 1080)

    def test_board_metadata_uses_spec_to_dict(self):
        self._write_sample()
        data = json.loads(self.tmp_path.read_text())
        self.assertEqual(data["board"], CHARUCO_BOARD.to_dict())

    def test_camera_serial_passed_through(self):
        self._write_sample(camera_serial="SN-FROM-CONFIG")
        data = json.loads(self.tmp_path.read_text())
        self.assertEqual(data["camera_serial"], "SN-FROM-CONFIG")

    def test_camera_serial_none_allowed(self):
        """No serial configured shouldn't crash the writer."""
        self._write_sample(camera_serial=None)
        data = json.loads(self.tmp_path.read_text())
        self.assertIsNone(data["camera_serial"])

    def test_captured_at_is_iso8601(self):
        self._write_sample()
        data = json.loads(self.tmp_path.read_text())
        # Should parse back as datetime; raises ValueError otherwise.
        parsed = datetime.fromisoformat(data["captured_at"])
        self.assertIsNotNone(parsed)

    def test_tool_version_present(self):
        self._write_sample()
        data = json.loads(self.tmp_path.read_text())
        self.assertTrue(data["tool_version"].startswith("M0b"))

    def test_overwrites_existing_artifact(self):
        """Re-solving replaces the previous calibration in place."""
        self._write_sample(rms_px=1.5)
        self._write_sample(rms_px=0.3)
        data = json.loads(self.tmp_path.read_text())
        self.assertAlmostEqual(data["rms_px"], 0.3)

    def test_creates_parent_directory_if_missing(self):
        nested = Path(self._tmp.name) / "nested" / "subdir" / "intrinsics.json"
        self._write_sample(target_path=nested)
        self.assertTrue(nested.exists())


if __name__ == "__main__":
    unittest.main()
