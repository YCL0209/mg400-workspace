"""M0a smoke test: open the Delta camera, grab one frame, save it as PNG.

Operator-facing CLI — uses ``print`` for status (per CLAUDE.md scripts
exception). Requires Windows + Delta DMV installer (DmvSDK on PYTHONPATH);
see docs/dmv_sdk.md.

Defaults to opening the camera whose serial matches ``viz.camera_serial``
in ``config/robot.json``; ``--serial`` and ``--device-index`` override
for ad-hoc identification (e.g. physical-cover test to figure out which
of two same-model cameras is the eye-in-hand one).

Usage::

    python -m robot_core.scripts.camera_smoke
    python -m robot_core.scripts.camera_smoke --serial C1M6GM0D23160059
    python -m robot_core.scripts.camera_smoke --device-index 0

Output: ``outputs/camera_smoke_<unix-ts>_<id>.png`` (1280×960 RGB). The
filename includes a tag identifying which camera was selected so back-to-back
captures don't clobber each other.
"""

from __future__ import annotations

import argparse
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


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Grab one frame from the Delta camera and save it as PNG."
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--serial",
        help="Open the camera with this serial (overrides config).",
    )
    group.add_argument(
        "--device-index",
        type=int,
        help="Open the Nth enumerated camera (debug fallback).",
    )
    return parser.parse_args()


def main() -> int:
    if not HAS_DMV_SDK:
        print(
            "ERROR: DmvSDK not importable. This script requires Windows with "
            "the Delta DMV installer; see docs/dmv_sdk.md.",
            file=sys.stderr,
        )
        return 2

    args = _parse_args()
    _OUTPUT_DIR.mkdir(exist_ok=True)

    # Selection priority: CLI flag > config serial > default first device.
    if args.serial is not None:
        kwargs = {"serial": args.serial}
        tag = f"serial-{args.serial}"
    elif args.device_index is not None:
        kwargs = {"device_index": args.device_index}
        tag = f"index-{args.device_index}"
    else:
        cfg_serial = _load_camera_serial()
        if cfg_serial:
            kwargs = {"serial": cfg_serial}
            tag = f"serial-{cfg_serial}"
        else:
            kwargs = {}
            tag = "default"

    out_path = _OUTPUT_DIR / f"camera_smoke_{int(time.time())}_{tag}.png"

    print("=" * 60)
    print("DMV-SDK camera smoke test (M0a)")
    print(f"  selection: {tag}")
    print("=" * 60)

    with DeltaCamera(**kwargs) as cam:
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
