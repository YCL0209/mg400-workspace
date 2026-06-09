"""Write the camera-intrinsics calibration artifact JSON.

The artifact (``config/camera_intrinsics.json``) is the durable handoff
from the M0b solver to M0c hand-eye sampling and the M2 FOV reprojection.
Schema is defined in PHASE2 design §8.1.4 and re-validated by
``tests/test_calib_artifact.py``.

Overwrites any existing file in place -- the latest calibration is the
source of truth for the current lens setup. Operators who need to keep
history should copy the file aside before re-solving; we don't auto-
rotate because the artifact is small and there's no use case yet.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import numpy as np

from robot_core.calibration.charuco import CharucoSpec

_DEFAULT_CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
_DEFAULT_PATH = _DEFAULT_CONFIG_DIR / "camera_intrinsics.json"
_TOOL_VERSION = "M0b-v1"


def write_artifact(
    *,
    K,
    dist,
    rms_px: float,
    image_size: tuple,  # (width, height) per cv2 convention
    n_views: int,
    board_spec: CharucoSpec,
    camera_serial: Optional[str],
    target_path: Optional[Path] = None,
) -> Path:
    """Write camera intrinsics to ``config/camera_intrinsics.json`` (or override).

    Coerces ndarray inputs to plain Python lists so the artifact is
    operator-readable JSON. ``dist`` is flattened to a flat list (cv2
    returns it as a column vector).

    Returns the path written. Caller (M0b solver) logs this so the
    operator sees where the file landed.
    """
    if target_path is None:
        target_path = _DEFAULT_PATH

    K_list = np.asarray(K, dtype=float).tolist()
    dist_list = np.asarray(dist, dtype=float).flatten().tolist()
    width, height = int(image_size[0]), int(image_size[1])

    payload = {
        "K": K_list,
        "dist": dist_list,
        "image_width": width,
        "image_height": height,
        "rms_px": float(rms_px),
        "n_views": int(n_views),
        "board": board_spec.to_dict(),
        "camera_serial": camera_serial,
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "tool_version": _TOOL_VERSION,
    }

    target_path.parent.mkdir(parents=True, exist_ok=True)
    target_path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return target_path


__all__ = ["write_artifact"]
