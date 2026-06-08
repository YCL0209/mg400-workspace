"""M0a smoke test: open the Delta camera, grab one frame, save it as PNG.

Operator-facing CLI — uses ``print`` for status (per CLAUDE.md scripts
exception). Requires Windows + Delta DMV installer (DmvSDK on PYTHONPATH);
see docs/dmv_sdk.md.

Reads ``viz.camera_serial`` from ``config/robot.json`` to disambiguate when
multiple same-model cameras share a hub. ``null`` (the default) opens the
first enumerated device; run ``python -m robot_core.scripts.list_cameras``
to discover serials.

Usage::

    python -m robot_core.scripts.camera_smoke

Output: ``outputs/camera_smoke_<unix-ts>.png`` (1280×960 RGB). Reports shape
and dtype to stdout so the operator can confirm the capture worked end to end.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

from PIL import Image

from robot_core.camera import HAS_DMV_SDK, DeltaCamera

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
_OUTPUT_DIR = _REPO_ROOT / "outputs"
_ROBOT_JSON = _REPO_ROOT / "config" / "robot.json"


def _load_camera_serial() -> str | None:
    """Read ``viz.camera_serial`` from config; ``null`` / missing → None."""
    try:
        with open(_ROBOT_JSON, encoding="utf-8") as fh:
            return json.load(fh).get("viz", {}).get("camera_serial")
    except FileNotFoundError:
        return None


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
    serial = _load_camera_serial()

    print("=" * 60)
    print("DMV-SDK camera smoke test (M0a)")
    if serial:
        print(f"  selecting camera by serial: {serial}")
    else:
        print("  no serial configured — opening first enumerated device")
    print("=" * 60)

    with DeltaCamera(serial=serial) as cam:
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
