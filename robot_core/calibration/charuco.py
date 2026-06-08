"""ChArUco calibration board specification -- single source of truth.

The board geometry is referenced by:

- ``robot_core/scripts/charuco_print.py``  -- generates the printable image
- ``viz/calib_session.py``                 -- M0b-2: detect + accumulate samples
- ``viz/calib_session.py.solve()``         -- M0b-4: cv2.aruco.calibrateCameraCharuco
- ``viz/calib_artifact.py``                -- M0b-4: write board metadata into
                                              ``config/camera_intrinsics.json``
- Future ``handeye_calib.py``              -- M0c: same board for hand-eye samples

Changing the physical board means a new artifact; bump ``tool_version`` and
re-calibrate. DICT_4X4_50 chosen to match phase5-panel's ArUco dictionary so
the two projects don't collide on marker IDs when both run on the same station.

cv2 conventions: square / marker sizes are passed in METERS (CharucoBoard ctor
expects SI units), but the spec keeps mm because that's what humans + the
printed artifact use. ``make_board()`` converts at the boundary.
"""

from __future__ import annotations

from dataclasses import dataclass

try:
    import cv2
    import cv2.aruco as aruco

    HAS_CV2 = True
except ImportError:
    cv2 = None  # type: ignore[assignment]
    aruco = None  # type: ignore[assignment]
    HAS_CV2 = False


_CV2_MISSING_MSG = (
    "cv2.aruco not available -- install opencv-contrib-python>=4.10 "
    "(regular opencv-python lacks the aruco submodule; do not install both "
    "side by side, they conflict)."
)


@dataclass(frozen=True)
class CharucoSpec:
    """Physical + cv2 metadata for a ChArUco board.

    Dimensions in millimeters; ``make_board()`` converts to meters for cv2.

    Default sizing: 7x10 squares x 20 mm = 140x200 mm board, fits A4
    portrait with a 20 mm white margin on every side (total 180x240 mm).
    A4 is the practical default in offices / labs without A3 printers.
    54 chess corners + DICT_4X4_50 markers is enough to solve K for a
    1440x1080 sensor at 20-40 cm working distance (each 20 mm square
    covers ~75 px on the sensor at 30 cm with an 8 mm lens). Calibration
    accuracy at this size is comparable to A3 because corner count and
    marker dictionary are unchanged -- only the physical extent shrinks.
    """

    squares_x: int = 7
    squares_y: int = 10
    square_size_mm: float = 20.0
    marker_size_mm: float = 15.0
    dictionary_name: str = "DICT_4X4_50"

    def to_dict(self) -> dict:
        """Serialise for writing into ``config/camera_intrinsics.json``."""
        return {
            "squares_x": self.squares_x,
            "squares_y": self.squares_y,
            "square_size_mm": self.square_size_mm,
            "marker_size_mm": self.marker_size_mm,
            "dictionary": self.dictionary_name,
        }


CHARUCO_BOARD = CharucoSpec()


def get_dictionary():
    """Return the cv2.aruco predefined dictionary for :data:`CHARUCO_BOARD`.

    Raises :class:`RuntimeError` if cv2.aruco is absent or the named
    dictionary isn't in the installed opencv-contrib version.
    """
    if not HAS_CV2:
        raise RuntimeError(_CV2_MISSING_MSG)
    try:
        const = getattr(aruco, CHARUCO_BOARD.dictionary_name)
    except AttributeError:
        raise RuntimeError(
            f"cv2.aruco missing dictionary constant "
            f"{CHARUCO_BOARD.dictionary_name!r} -- upgrade opencv-contrib-python"
        )
    return aruco.getPredefinedDictionary(const)


def make_board():
    """Construct the canonical :class:`cv2.aruco.CharucoBoard`.

    Uses the opencv-contrib-python >=4.7 ctor signature
    ``CharucoBoard((cols, rows), squareLength, markerLength, dictionary)``.
    requirements.txt pins >=4.10 so we don't carry a fallback for the
    older ``CharucoBoard_create(...)`` API.
    """
    if not HAS_CV2:
        raise RuntimeError(_CV2_MISSING_MSG)
    spec = CHARUCO_BOARD
    return aruco.CharucoBoard(
        (spec.squares_x, spec.squares_y),
        spec.square_size_mm / 1000.0,  # cv2 expects metres
        spec.marker_size_mm / 1000.0,
        get_dictionary(),
    )


__all__ = [
    "CharucoSpec",
    "CHARUCO_BOARD",
    "HAS_CV2",
    "get_dictionary",
    "make_board",
]
