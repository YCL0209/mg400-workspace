"""Enumerate all connected DMV cameras with their serials.

Run this once on a Windows box to find out which camera is your eye-in-hand
unit when several same-model cameras share a USB/GigE hub. Copy the printed
serial into ``config/robot.json`` under ``viz.camera_serial`` so subsequent
camera_smoke / calibration / inspection scripts open the right one
deterministically.

Usage::

    python -m robot_core.scripts.list_cameras
"""

from __future__ import annotations

import sys

from robot_core.camera import HAS_DMV_SDK, DeltaCamera


def main() -> int:
    if not HAS_DMV_SDK:
        print(
            "ERROR: DmvSDK not importable. Requires Windows + Delta DMV "
            "installer; see docs/dmv_sdk.md.",
            file=sys.stderr,
        )
        return 2

    try:
        devices = DeltaCamera.list_devices()
    except RuntimeError as e:
        print(f"ERROR enumerating cameras: {e}", file=sys.stderr)
        return 3

    if not devices:
        print("No cameras detected.")
        return 1

    print(f"Found {len(devices)} camera(s):\n")
    for d in devices:
        # flat_index is what to pass as DeltaCamera(device_index=...) for
        # debug; serial is the durable identifier for config.
        header = f"[flat={d.get('flat_index', '?')}]  iface={d.get('interface_index', '?')} dev={d.get('device_index', '?')}"
        print(f"  {header}")
        for field in (
            "display_name",
            "model",
            "vendor",
            "version",
            "serial",
            "user_defined_name",
            "error",
        ):
            if field in d:
                print(f"      {field:20s} {d[field]}")
        print()

    print("Copy the desired serial into config/robot.json:")
    print('  "viz": { ..., "camera_serial": "<paste here>" }')
    return 0


if __name__ == "__main__":
    sys.exit(main())
