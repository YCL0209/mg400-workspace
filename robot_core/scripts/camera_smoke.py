"""M0a smoke test: open the Delta camera, grab one frame, save it as PNG.

Operator-facing CLI — uses ``print`` for status (per CLAUDE.md scripts
exception). Requires Windows + Delta DMV installer (DmvSDK on PYTHONPATH);
see docs/dmv_sdk.md.

Usage::

    python -m robot_core.scripts.camera_smoke

Output: ``outputs/camera_smoke_<unix-ts>.png`` (1280×960 RGB). Reports shape
and dtype to stdout so the operator can confirm the capture worked end to end.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

from PIL import Image

from robot_core.camera import HAS_DMV_SDK, DeltaCamera

_OUTPUT_DIR = Path(__file__).resolve().parent.parent.parent / "outputs"


def main() -> int:
    if not HAS_DMV_SDK:
        print(
            "ERROR: DmvSDK not importable. This script requires Windows with "
            "the Delta DMV installer; see docs/dmv_sdk.md.",
            file=sys.stderr,
        )
        return 2

    _OUTPUT_DIR.mkdir(exist_ok=True)
    out_path = _OUTPUT_DIR / f"camera_smoke_{int(time.time())}.png"

    print("=" * 60)
    print("DMV-SDK camera smoke test (M0a)")
    print("=" * 60)

    with DeltaCamera() as cam:
        print("\ncapturing one frame...")
        rgb = cam.grab_one_rgb()
        print(f"got ndarray: shape={rgb.shape} dtype={rgb.dtype}")

    Image.fromarray(rgb).save(out_path)
    size_kb = out_path.stat().st_size / 1024
    print(f"\nsaved: {out_path} ({size_kb:.1f} KB)")
    print("=" * 60)
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
