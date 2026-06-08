"""Generate the ChArUco calibration board as a printable PNG.

Run::

    python -m robot_core.scripts.charuco_print

Outputs ``outputs/charuco_board.png`` sized for A3 printing at 300 DPI.
The board image is centered on a white canvas with a margin for cutting +
mounting on rigid backing without clipping markers. **Print at 100% scale
(no "fit-to-page")** so the printed square edges match the spec in
``robot_core/calibration/charuco.py`` -- the calibration solver relies on
those physical dimensions for K's unit interpretation.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

try:
    import cv2

    HAS_CV2 = True
except ImportError:
    cv2 = None  # type: ignore[assignment]
    HAS_CV2 = False

from robot_core.calibration.charuco import CHARUCO_BOARD, HAS_CV2 as _HAS_ARUCO, make_board

_OUTPUT_DIR = Path(__file__).resolve().parent.parent.parent / "outputs"
_PRINT_DPI = 300
_MM_PER_INCH = 25.4
_MARGIN_MM = 20  # white border for cutting / mounting


def _mm_to_px(mm: float, dpi: int = _PRINT_DPI) -> int:
    """Convert physical mm to pixels at the configured print DPI."""
    return int(round(mm / _MM_PER_INCH * dpi))


def main() -> int:
    if not (HAS_CV2 and _HAS_ARUCO):
        print(
            "ERROR: cv2.aruco not importable. Install opencv-contrib-python "
            ">=4.10 (regular opencv-python lacks the aruco submodule).",
            file=sys.stderr,
        )
        return 2

    spec = CHARUCO_BOARD
    board = make_board()

    board_w_mm = spec.squares_x * spec.square_size_mm
    board_h_mm = spec.squares_y * spec.square_size_mm
    total_w_mm = board_w_mm + 2 * _MARGIN_MM
    total_h_mm = board_h_mm + 2 * _MARGIN_MM

    board_w_px = _mm_to_px(board_w_mm)
    board_h_px = _mm_to_px(board_h_mm)
    margin_px = _mm_to_px(_MARGIN_MM)
    total_w_px = board_w_px + 2 * margin_px
    total_h_px = board_h_px + 2 * margin_px

    # Render the board itself at the exact physical pixel size.
    board_img = board.generateImage((board_w_px, board_h_px))

    # Centre on a white canvas with margin for clean cutting.
    canvas = np.full((total_h_px, total_w_px), 255, dtype=np.uint8)
    canvas[margin_px : margin_px + board_h_px, margin_px : margin_px + board_w_px] = (
        board_img
    )

    _OUTPUT_DIR.mkdir(exist_ok=True)
    out_path = _OUTPUT_DIR / "charuco_board.png"
    cv2.imwrite(str(out_path), canvas)

    print("=" * 60)
    print("ChArUco board generated for printing")
    print("=" * 60)
    print(f"  spec:           {spec.squares_x} x {spec.squares_y} squares")
    print(
        f"                  square {spec.square_size_mm} mm, "
        f"marker {spec.marker_size_mm} mm"
    )
    print(f"                  dictionary {spec.dictionary_name}")
    print(f"  physical board: {board_w_mm:.0f} x {board_h_mm:.0f} mm")
    print(f"  with margins:   {total_w_mm:.0f} x {total_h_mm:.0f} mm")
    print(f"  pixels @ {_PRINT_DPI} DPI: {total_w_px} x {total_h_px}")
    print()
    print(f"  saved: {out_path}")
    print(f"  size:  {out_path.stat().st_size / 1024:.1f} KB")
    print("=" * 60)
    print(
        "Print at 100% scale (no fit-to-page) on A3 paper for correct "
        "dimensions; mount on rigid cardboard before use."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
