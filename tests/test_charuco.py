"""Tests for the ChArUco calibration board spec + factory.

Pure-Python spec tests run on any install. cv2-dependent tests
(``make_board()`` + ``get_dictionary()`` + ``generateImage()``) skip cleanly
when opencv-contrib-python isn't present so Mac dev without the camera
toolchain still gets a green run.
"""

import json
import unittest

from robot_core.calibration.charuco import CHARUCO_BOARD, HAS_CV2, CharucoSpec


class TestCharucoSpec(unittest.TestCase):
    """Pure-Python spec checks -- no cv2 needed."""

    def test_default_spec_matches_phase2_design_section_8_1_1(self):
        """Defaults must match the contract in PHASE2 design §8.1.1.

        These values are the source of truth referenced by every other
        calibration script + the artifact JSON; silent drift would
        invalidate already-calibrated K matrices.
        """
        spec = CHARUCO_BOARD
        self.assertEqual(spec.squares_x, 7)
        self.assertEqual(spec.squares_y, 10)
        self.assertAlmostEqual(spec.square_size_mm, 20.0)
        self.assertAlmostEqual(spec.marker_size_mm, 15.0)
        self.assertEqual(spec.dictionary_name, "DICT_4X4_50")

    def test_marker_is_smaller_than_square(self):
        """cv2.aruco requires the marker to fit inside its chess square."""
        spec = CHARUCO_BOARD
        self.assertLess(spec.marker_size_mm, spec.square_size_mm)

    def test_spec_is_frozen_dataclass(self):
        """Spec must be immutable -- accidental mutation would corrupt a calibration run."""
        with self.assertRaises(Exception):
            CHARUCO_BOARD.squares_x = 99  # type: ignore[misc]

    def test_to_dict_serialises_for_artifact(self):
        d = CHARUCO_BOARD.to_dict()
        for key in (
            "squares_x",
            "squares_y",
            "square_size_mm",
            "marker_size_mm",
            "dictionary",
        ):
            self.assertIn(key, d)
        # JSON round-trip must be lossless -- artifact storage is JSON.
        self.assertEqual(json.loads(json.dumps(d)), d)

    def test_custom_spec_is_still_a_valid_dataclass(self):
        custom = CharucoSpec(squares_x=5, squares_y=7, square_size_mm=30.0)
        self.assertEqual(custom.squares_x, 5)
        # Marker size default kept; future scripts should override consistently.
        self.assertEqual(custom.marker_size_mm, 15.0)


@unittest.skipUnless(HAS_CV2, "opencv-contrib-python not installed")
class TestCharucoBoardFactory(unittest.TestCase):
    """cv2.aruco-dependent tests."""

    def test_make_board_returns_charuco_board(self):
        import cv2

        from robot_core.calibration.charuco import make_board

        board = make_board()
        self.assertIsInstance(board, cv2.aruco.CharucoBoard)

    def test_get_dictionary_returns_predefined_dictionary(self):
        import cv2

        from robot_core.calibration.charuco import get_dictionary

        d = get_dictionary()
        self.assertIsInstance(d, cv2.aruco.Dictionary)

    def test_generate_image_at_requested_size(self):
        """generateImage((w, h)) must yield ``(h, w) uint8`` ndarray ready for print."""
        from robot_core.calibration.charuco import make_board

        board = make_board()
        img = board.generateImage((700, 1000))
        self.assertEqual(img.shape, (1000, 700))
        self.assertEqual(img.dtype.name, "uint8")


if __name__ == "__main__":
    unittest.main()
